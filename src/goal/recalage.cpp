/**
 * Recalage automatique sur bordure.
 *
 * Principe : plaqué contre un mur, on connaît le cap du robot (perpendiculaire au mur) et UNE
 * coordonnée (celle du mur ± distance base_link -> face de contact). On envoie ces valeurs au
 * service correct_pose (map_odom_publisher) qui corrige map -> odom ; l'autre coordonnée est
 * conservée.
 *
 * Toutes ces fonctions sont BLOQUANTES (usleep) : elles sont appelées depuis loop(), comme les
 * autres mouvements de GoalStrat. Le service est appelé en asynchrone et on attend le future
 * avec un timeout. Le client est dans son propre callback group : le MultiThreadedExecutor peut
 * donc traiter la réponse sur un autre thread pendant que loop() attend.
 */

#include "goal_strategy/goal.h"
#include "goal_strategy/goal_logic.h"

#include <algorithm>
#include <cmath>
#include <unistd.h>

using namespace std::chrono_literals;

namespace
{
constexpr double kTableHalfX = 1.5; // m
constexpr double kTableHalfY = 1.0; // m

/// Direction (cap) qui pointe du mur vers l'intérieur de la table
double inwardHeading(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return 0.0;
    case Wall::X_MAX:
        return M_PI;
    case Wall::Y_MIN:
        return M_PI / 2;
    case Wall::Y_MAX:
    default:
        return -M_PI / 2;
    }
}

/// Coordonnée du mur dans map
double wallCoordinate(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return -kTableHalfX;
    case Wall::X_MAX:
        return kTableHalfX;
    case Wall::Y_MIN:
        return -kTableHalfY;
    case Wall::Y_MAX:
    default:
        return kTableHalfY;
    }
}

bool isXWall(Wall a_wall)
{
    return a_wall == Wall::X_MIN || a_wall == Wall::X_MAX;
}

const char* wallName(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return "X_MIN";
    case Wall::X_MAX:
        return "X_MAX";
    case Wall::Y_MIN:
        return "Y_MIN";
    case Wall::Y_MAX:
    default:
        return "Y_MAX";
    }
}
} // namespace

/**
 * @brief Déclare les paramètres ROS du recalage, crée le client correct_pose et l'abonnement
 * motors_current. Appelé depuis le constructeur.
 */
void GoalStrat::declareRecalageParameters()
{
    // Distance base_link -> bord arrière, mesurée sur le robot : 104 mm (URDF simu : 105 mm).
    // Les roues d'odométrie sont distinctes des roues de propulsion : quand le robot bute,
    // l'odom s'arrête même si les roues motrices patinent => détection "immobile" fiable.
    // En simu, krabi_description/scripts/sim_odometry.py reproduit ce comportement.
    m_recalage.contact_offset_back = this->declare_parameter("recalage.contact_offset_back", 0.104);
    // Face avant : non mesurée (contact avant non utilisé pour l'instant)
    m_recalage.contact_offset_front
      = this->declare_parameter("recalage.contact_offset_front", 0.104);
    m_recalage.still_dist = this->declare_parameter("recalage.still_dist", 0.002);
    m_recalage.still_time = this->declare_parameter("recalage.still_time", 0.3);
    m_recalage.settle_time = this->declare_parameter("recalage.settle_time", 0.2);
    m_recalage.timeout = this->declare_parameter("recalage.timeout", 5.0);
    m_recalage.align_timeout = this->declare_parameter("recalage.align_timeout", 5.0);
    m_recalage.service_timeout = this->declare_parameter("recalage.service_timeout", 1.0);
    m_recalage.clearance = this->declare_parameter("recalage.clearance", 0.10);
    // Garde-fous : un robot qui ne démarre pas est immobile, sans être contre un mur
    m_recalage.min_push_dist = this->declare_parameter("recalage.min_push_dist", 0.02);
    m_recalage.max_correction_dist = this->declare_parameter("recalage.max_correction_dist", 0.15);
    m_recalage.max_correction_angle
      = this->declare_parameter("recalage.max_correction_angle", 0.35); // 20°
    m_recalage.use_lidar_loc = this->declare_parameter("recalage.use_lidar_loc", false);

    // Groupe dédié : la réponse du service et motors_current sont traités même si loop()
    // (groupe par défaut) est bloqué dans un recalage.
    m_recalage_cb_group
      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_correct_pose_client = this->create_client<krabi_msgs::srv::CorrectPose>(
      "correct_pose", rclcpp::ServicesQoS(), m_recalage_cb_group);

    rclcpp::SubscriptionOptions l_options;
    l_options.callback_group = m_recalage_cb_group;
    m_motors_current_sub = this->create_subscription<krabi_msgs::msg::MotorsCurrent>(
      "motors_current",
      5,
      std::bind(&GoalStrat::updateMotorsCurrent, this, std::placeholders::_1),
      l_options);

    // ---- Actions avant match ----
    // Pose de départ (map), passée par le launch (= xRobotPos / yRobotPos / zRobotOrientation)
    m_start_pose_x = this->declare_parameter("start_pose.x", -1.25);
    m_start_pose_y = this->declare_parameter("start_pose.y", -0.75);
    m_start_pose_theta = this->declare_parameter("start_pose.theta", M_PI / 2);

    // Client vers Core (main_strategy) : autorise les moteurs avant la tirette
    m_pre_match_motion_client = this->create_client<std_srvs::srv::SetBool>(
      "enable_pre_match_motion", rclcpp::ServicesQoS(), m_recalage_cb_group);

    // Les services ne font que déposer une demande : l'action s'exécute dans loop()
    m_start_recalage_srv = this->create_service<std_srvs::srv::Trigger>(
      "start_recalage",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      { requestPreMatchAction(PRE_MATCH_RECALAGE, res); },
      rclcpp::ServicesQoS(),
      m_recalage_cb_group);
    m_go_to_start_srv = this->create_service<std_srvs::srv::Trigger>(
      "go_to_start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      { requestPreMatchAction(PRE_MATCH_GO_TO_START, res); },
      rclcpp::ServicesQoS(),
      m_recalage_cb_group);
}

/**
 * @brief Callback commun de start_recalage / go_to_start. N'accepte la demande qu'avant le
 * match (état WAIT_TIRETTE) et si aucune autre action n'est en cours.
 */
void GoalStrat::requestPreMatchAction(PreMatchRequest a_request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> a_res)
{
    if (m_state != State::WAIT_TIRETTE)
    {
        a_res->success = false;
        a_res->message = "refusé : uniquement avant le match (état WAIT_TIRETTE)";
    }
    else if (m_pre_match_busy || m_pre_match_request != PRE_MATCH_NONE)
    {
        a_res->success = false;
        a_res->message = "refusé : une action avant match est déjà en cours";
    }
    else
    {
        m_pre_match_request = a_request;
        a_res->success = true;
        a_res->message = "accepté, voir les logs de goal_strat pour le résultat";
    }
    RCLCPP_INFO(this->get_logger(), "%s", a_res->message.c_str());
}

/**
 * @brief Exécute l'action avant match demandée, s'il y en a une. Appelé depuis loop().
 * @return true si une action a été exécutée
 */
bool GoalStrat::handlePreMatchRequest()
{
    const int l_request = m_pre_match_request.load();
    if (l_request == PRE_MATCH_NONE)
    {
        return false;
    }

    m_pre_match_busy = true;
    m_pre_match_request = PRE_MATCH_NONE;

    if (setPreMatchMotion(true))
    {
        bool l_ok = false;
        if (l_request == PRE_MATCH_RECALAGE)
        {
            l_ok = recalageInitial();
        }
        else if (l_request == PRE_MATCH_GO_TO_START)
        {
            l_ok = goToStartPose();
        }
        RCLCPP_INFO(this->get_logger(),
                    "Action avant match %s : %s",
                    l_request == PRE_MATCH_RECALAGE ? "recalage" : "go_to_start",
                    l_ok ? "réussie" : "ÉCHEC");
        // Quel que soit le résultat : ne rien laisser traîner pour le match (vitesse forcée,
        // sens de marche imposé par override_gear...)
        stopPreMatchMotion();
        setPreMatchMotion(false);
    }

    m_pre_match_busy = false;
    return true;
}

/**
 * @brief Active/désactive les moteurs avant match côté Core (service enable_pre_match_motion).
 */
bool GoalStrat::setPreMatchMotion(bool a_enable)
{
    const auto l_timeout = std::chrono::duration<double>(m_recalage.service_timeout);
    if (!m_pre_match_motion_client->wait_for_service(l_timeout))
    {
        RCLCPP_ERROR(this->get_logger(), "Service enable_pre_match_motion indisponible");
        return false;
    }
    auto l_req = std::make_shared<std_srvs::srv::SetBool::Request>();
    l_req->data = a_enable;
    auto l_future = m_pre_match_motion_client->async_send_request(l_req);
    if (l_future.wait_for(l_timeout) != std::future_status::ready)
    {
        m_pre_match_motion_client->remove_pending_request(l_future);
        RCLCPP_ERROR(this->get_logger(), "Pas de réponse de enable_pre_match_motion");
        return false;
    }
    return l_future.get()->success;
}

/**
 * @brief Arrête le robot sur place et rend la main à la stratégie (sens de marche libre).
 */
void GoalStrat::stopPreMatchMotion()
{
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    startAngular(); // quitte RECALAGE_BORDURE
    publishStratMovement();
}

/**
 * @brief Attend, en continuant de publier l'ordre courant (Core coupe les moteurs avant match
 * s'il ne reçoit plus rien).
 * @return false si la tirette a été retirée entre-temps (le match démarre)
 */
bool GoalStrat::sleepUnlessMatch(double a_seconds)
{
    const double l_end = TiretteFilter::steadyNow() + a_seconds;
    while (TiretteFilter::steadyNow() < l_end)
    {
        if (m_tirette_filter.started())
        {
            return false;
        }
        publishStratMovement();
        usleep(50000);
    }
    return !m_tirette_filter.started();
}

/**
 * @brief Date de la dernière mesure d'odométrie (odom -> base_link).
 */
bool GoalStrat::getOdomStamp(rclcpp::Time& a_stamp)
{
    try
    {
        const auto l_tf = m_tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        a_stamp = rclcpp::Time(l_tf.header.stamp, this->now().get_clock_type());
        return true;
    }
    catch (const tf2::TransformException&)
    {
        return false;
    }
}

/**
 * @brief Va à la pose de départ (start_pose.*), puis s'oriente.
 */
bool GoalStrat::goToStartPose()
{
    const Position l_target{ Distance(m_start_pose_x), Distance(m_start_pose_y) };
    const Angle l_theta = Angle(m_start_pose_theta);
    RCLCPP_INFO(this->get_logger(),
                "go_to_start : (%.3f, %.3f, %.3f)",
                m_start_pose_x,
                m_start_pose_y,
                m_start_pose_theta);

    m_goal_pose.setPosition(l_target);
    m_goal_pose.setAngle(l_theta);
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    startAngular(); // orient = GO_TO_GOALPOSE_POSITION
    m_strat_mvnt.max_speed_at_arrival = 0.f;

    bool l_arrived = false;
    const auto l_deadline = this->now() + rclcpp::Duration(10, 0);
    while (this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();
        if ((m_current_pose.getPosition() - l_target).getNorme() < REACH_DIST)
        {
            l_arrived = true;
            break;
        }
    }
    if (!l_arrived)
    {
        RCLCPP_WARN(this->get_logger(), "go_to_start : position non atteinte");
        m_goal_pose = m_current_pose;
        publishStratMovement();
        return false;
    }

    const bool l_aligned = alignInPlace(l_theta, rclcpp::Duration(5, 0));
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    startAngular();
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::STOP_ANGULAR;
    publishStratMovement();
    RCLCPP_INFO_STREAM(this->get_logger(), "go_to_start terminé, pose : " << m_current_pose);
    return l_aligned;
}

void GoalStrat::updateMotorsCurrent(krabi_msgs::msg::MotorsCurrent::SharedPtr a_msg)
{
    // != 0 : la STM32 a détecté un calage et bloque la roue pendant ce nombre de ms
    m_left_wheel_stalled = a_msg->left_wheel_unstalled_in_ms != 0;
    m_right_wheel_stalled = a_msg->right_wheel_unstalled_in_ms != 0;
}

/**
 * @brief Tourne sur place jusqu'au cap a_angle (cap de base_link, marche avant).
 * Contrairement à alignWithAngle(), la position cible est la position courante : le robot ne
 * se déplace pas vers l'étape en cours.
 */
bool GoalStrat::alignInPlace(Angle a_angle, rclcpp::Duration a_timeout)
{
    updateCurrentPose();
    m_goal_pose.setPosition(m_current_pose.getPosition());
    m_goal_pose.setAngle(a_angle);

    // FORWARD : en mode orientation, Core vise goal_angle (+π si REVERSE). On veut le cap brut.
    override_gear = krabi_msgs::msg::StratMovement::FORWARD;
    startAngular();
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::ORIENT_TOWARD_GOALPOSE_ORIENTATION;

    const auto l_deadline = this->now() + a_timeout;
    while (this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        updateCurrentPose();
        if (isAlignedWithAngle(a_angle))
        {
            return true;
        }
        usleep(50000);
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "alignInPlace: non aligné, cap " << m_current_pose.getAngle() << " au lieu de "
                                                     << a_angle);
    return false;
}

/**
 * @brief Attend le contact avec le mur pendant la poussée.
 * Contact = déplacement < still_dist pendant still_time, OU les deux roues calées (STM32),
 * et seulement si le robot a d'abord avancé d'au moins min_push_dist : un robot qui ne démarre
 * pas (PWM trop faible, roues bloquées) est immobile sans être contre le mur.
 * L'immobilité est mesurée sur les dates des mesures d'odométrie, pas sur l'horloge : si
 * l'odométrie s'arrête (CAN coupé...), la pose se fige mais aucun contact n'est déclaré.
 * @return false si timeout, odométrie perdue ou tirette retirée
 */
bool GoalStrat::waitForWallContact(rclcpp::Duration a_timeout)
{
    const auto l_deadline = this->now() + a_timeout;

    updateCurrentPose();
    const Position l_start_position = m_current_pose.getPosition();

    // On ignore le tout début : le robot doit d'abord se mettre en mouvement
    if (!sleepUnlessMatch(m_recalage.still_time))
    {
        return false;
    }

    updateCurrentPose();
    Position l_ref_position = m_current_pose.getPosition();
    rclcpp::Time l_odom_stamp = this->now();
    getOdomStamp(l_odom_stamp);
    rclcpp::Time l_ref_time = l_odom_stamp;

    bool l_has_moved = false;
    while (this->now() < l_deadline)
    {
        if (m_tirette_filter.started())
        {
            return false;
        }
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();

        if (!getOdomStamp(l_odom_stamp) || (this->now() - l_odom_stamp).seconds() > 0.5)
        {
            RCLCPP_ERROR(this->get_logger(), "Recalage: odométrie perdue, abandon");
            return false;
        }

        l_has_moved = l_has_moved
                      || (m_current_pose.getPosition() - l_start_position).getNorme()
                           >= m_recalage.min_push_dist;

        if (l_has_moved && m_left_wheel_stalled && m_right_wheel_stalled)
        {
            RCLCPP_INFO(this->get_logger(), "Recalage: contact détecté (roues calées)");
            return true;
        }

        const double l_moved = (m_current_pose.getPosition() - l_ref_position).getNorme();
        if (l_moved > m_recalage.still_dist)
        {
            // Le robot bouge encore : on repart de cette position
            l_ref_position = m_current_pose.getPosition();
            l_ref_time = l_odom_stamp;
        }
        else if (l_has_moved && (l_odom_stamp - l_ref_time).seconds() >= m_recalage.still_time)
        {
            RCLCPP_INFO(this->get_logger(), "Recalage: contact détecté (robot immobile)");
            return true;
        }
    }
    RCLCPP_WARN(this->get_logger(),
                l_has_moved ? "Recalage: timeout, pas de contact détecté"
                            : "Recalage: le robot n'a pas bougé en poussant (déjà contre le mur ? "
                              "roues bloquées ? PWM trop faible ?)");
    return false;
}

/**
 * @brief Appelle correct_pose (asynchrone + attente avec timeout).
 */
bool GoalStrat::callCorrectPose(Wall a_wall, Angle a_theta, Distance a_coord)
{
    if (!m_correct_pose_client->wait_for_service(std::chrono::duration<double>(
          m_recalage.service_timeout)))
    {
        RCLCPP_ERROR(this->get_logger(), "Recalage: service correct_pose indisponible");
        return false;
    }

    auto l_req = std::make_shared<krabi_msgs::srv::CorrectPose::Request>();
    l_req->theta = static_cast<double>(a_theta);
    l_req->set_theta = true;
    if (isXWall(a_wall))
    {
        l_req->x = static_cast<double>(a_coord);
        l_req->set_x = true;
    }
    else
    {
        l_req->y = static_cast<double>(a_coord);
        l_req->set_y = true;
    }

    auto l_future = m_correct_pose_client->async_send_request(l_req);
    // Pas de spin_until_future_complete ici (on est dans un callback) : l'executor
    // multithread traite la réponse dans m_recalage_cb_group.
    if (l_future.wait_for(std::chrono::duration<double>(m_recalage.service_timeout))
        != std::future_status::ready)
    {
        m_correct_pose_client->remove_pending_request(l_future);
        RCLCPP_ERROR(this->get_logger(), "Recalage: pas de réponse de correct_pose");
        return false;
    }

    auto l_res = l_future.get();
    if (!l_res->success)
    {
        RCLCPP_ERROR(this->get_logger(), "Recalage: correct_pose a échoué: %s", l_res->message.c_str());
        return false;
    }
    return true;
}

/**
 * @brief Recalage sur un mur.
 * @param a_wall le mur visé
 * @param a_reverse true = contact par l'arrière (le robot recule vers le mur)
 */
bool GoalStrat::recalageBordure(Wall a_wall, bool a_reverse)
{
    if (m_recalage.use_lidar_loc)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Recalage désactivé en mode use_lidar_loc (AMCL publie map->odom)");
        return false;
    }

    // Cap au contact : dos au mur => on regarde vers l'intérieur de la table
    const double l_inward = inwardHeading(a_wall);
    const Angle l_heading = AngleTools::wrapAngle(Angle(a_reverse ? l_inward : l_inward + M_PI));
    const double l_offset
      = a_reverse ? m_recalage.contact_offset_back : m_recalage.contact_offset_front;
    // base_link est à l_offset du mur, du côté intérieur
    const double l_inward_sign = (a_wall == Wall::X_MIN || a_wall == Wall::Y_MIN) ? 1.0 : -1.0;
    const Distance l_coord = Distance(wallCoordinate(a_wall) + l_inward_sign * l_offset);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Recalage sur " << wallName(a_wall) << (a_reverse ? " (arrière)" : " (avant)")
                                       << " : cap visé " << l_heading << ", "
                                       << (isXWall(a_wall) ? "x" : "y") << " = " << l_coord);

    // 1. Alignement perpendiculaire au mur
    if (!alignInPlace(l_heading, rclcpp::Duration::from_seconds(m_recalage.align_timeout)))
    {
        return false;
    }

    // 2. Poussée (Core : PWM fixe + courant limité ; en simu, petite vitesse cmd_vel)
    m_left_wheel_stalled = false;
    m_right_wheel_stalled = false;
    recalage_bordure();
    override_gear = a_reverse ? krabi_msgs::msg::StratMovement::REVERSE
                              : krabi_msgs::msg::StratMovement::FORWARD;
    publishStratMovement();

    // 3. Détection du contact
    const bool l_contact
      = waitForWallContact(rclcpp::Duration::from_seconds(m_recalage.timeout));

    if (l_contact)
    {
        // 4. On laisse le robot se plaquer (on pousse toujours)
        sleepUnlessMatch(m_recalage.settle_time);
    }

    // Fin de poussée AVANT la correction : si on corrigeait en poussant encore, l'odom
    // continuerait de bouger (patinage en simu) entre la correction et l'arrêt effectif.
    // Cible = pose courante, rotation bloquée => le robot s'arrête contre le mur.
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    startAngular();
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::STOP_ANGULAR;
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    publishStratMovement();

    bool l_corrected = false;
    // Attente que Core (boucle 100 ms) ait appliqué l'arrêt et que l'odom soit stable
    if (l_contact && sleepUnlessMatch(std::max(m_recalage.settle_time, 0.3)))
    {
        // Garde-fou : une grosse correction veut dire que ce n'est pas le bon mur, qu'un objet
        // a bloqué le robot, ou qu'il n'était pas là où on le croyait.
        updateCurrentPose();
        const double l_coord_error
          = std::abs(static_cast<double>(isXWall(a_wall) ? m_current_pose.getPosition().getX()
                                                         : m_current_pose.getPosition().getY())
                     - static_cast<double>(l_coord));
        const double l_angle_error = std::abs(
          static_cast<double>(AngleTools::diffAngle(l_heading, m_current_pose.getAngle())));
        if (l_coord_error > m_recalage.max_correction_dist
            || l_angle_error > m_recalage.max_correction_angle)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Recalage: correction refusée, trop grande (%.3f m, %.3f rad)",
                         l_coord_error,
                         l_angle_error);
            return false;
        }

        // 5. Correction de map -> odom
        l_corrected = callCorrectPose(a_wall, l_heading, l_coord);

        // La pose map a sauté : nouvelle pose courante comme cible, pour que Core ne reparte
        // pas vers la cible d'avant correction.
        usleep(100000); // laisse le temps au nouveau map->odom d'arriver dans le buffer TF
        updateCurrentPose();
        m_goal_pose = m_current_pose;
        publishStratMovement();
    }

    if (!l_corrected)
    {
        return false;
    }

    // 6. Dégagement : on s'éloigne du mur (sens opposé à la poussée), sans tourner.
    // (pas avanceDroit() : goToDroit() force override_gear = REVERSE)
    updateCurrentPose();
    Position l_target = goal_logic::straightLineTarget(m_current_pose.getPosition(),
                                                       m_current_pose.getAngle(),
                                                       Distance(m_recalage.clearance),
                                                       !a_reverse /* sensRecule */);
    m_goal_pose.setPosition(l_target);
    m_goal_pose.setAngle(m_current_pose.getAngle());
    override_gear = a_reverse ? krabi_msgs::msg::StratMovement::FORWARD
                              : krabi_msgs::msg::StratMovement::REVERSE;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;

    const auto l_deadline = this->now() + rclcpp::Duration(3, 0);
    while (this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();
        if ((m_current_pose.getPosition() - l_target).getNorme() < REACH_DIST)
        {
            break;
        }
    }
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    publishStratMovement();

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Recalage " << wallName(a_wall) << " terminé, pose: " << m_current_pose);
    return !m_tirette_filter.started();
}

/**
 * @brief Recalage dans le coin du nid : mur latéral x puis mur y = -1.
 * Tout est relatif à la pose courante (odom) : on ne vise aucune position absolue.
 * Symétrie (useXSymetry, voir positionC) : bleu = côté x < 0, jaune = côté x > 0.
 */
bool GoalStrat::recalageInitial()
{
    const Wall l_x_wall = m_is_blue ? Wall::X_MIN : Wall::X_MAX;

    RCLCPP_INFO(this->get_logger(), "Recalage initial (%s)", m_is_blue ? "bleu" : "jaune");
    if (!recalageBordure(l_x_wall, true))
    {
        RCLCPP_ERROR(this->get_logger(), "Recalage initial: échec sur le mur x");
        return false;
    }
    if (!recalageBordure(Wall::Y_MIN, true))
    {
        RCLCPP_ERROR(this->get_logger(), "Recalage initial: échec sur le mur y");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Recalage initial réussi");
    return true;
}
