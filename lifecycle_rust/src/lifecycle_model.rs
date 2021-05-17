use rosrust_actionlib::action_server::ServerSimpleGoalHandle;
use rosrust_actionlib::ActionServer;
use rosrust_msg::std_msgs::Header;
use rosrust_msg::lifecycle_msgs::Lifecycle;
use rosrust_msg::lifecycle_msgs::LifecycleAction;
use rosrust_msg::lifecycle_msgs::LifecycleFeedback;
use rosrust_msg::lifecycle_msgs::LifecycleGoal;
use rosrust_msg::lifecycle_msgs::LifecycleResult;

pub enum State {
    // Primary States
    UNCONFIGURED    = LifecycleGoal::PSTATE_UNCONFIGURED as isize,
    INACTIVE        = LifecycleGoal::PSTATE_INACTIVE as isize,
    ACTIVE          = LifecycleGoal::PSTATE_ACTIVE as isize,
    FINALIZED       = LifecycleGoal::PSTATE_FINALIZED as isize,
    // Secondary States
    ErrorProcessing = LifecycleGoal::TSTATE_ERROR_PROCESSING as isize,
    CleaningUp      = LifecycleGoal::TSTATE_CLEANING_UP as isize,
    Configuring     = LifecycleGoal::TSTATE_CONFIGURING as isize,
    Activating      = LifecycleGoal::TSTATE_ACTIVATING as isize,
    Deactivating    = LifecycleGoal::TSTATE_DEACTIVATING as isize,
    ShuttingDown    = LifecycleGoal::TSTATE_SHUTTING_DOWN as isize,
}

impl Default for State {
    fn default() -> Self { State::UNCONFIGURED }
}

#[derive(Debug)]
pub enum Transition {
    CONFIGURE   = LifecycleGoal::EV_CONFIGURE as isize,
    CLEANUP     = LifecycleGoal::EV_CLEANUP as isize,
    ACTIVATE    = LifecycleGoal::EV_ACTIVATE as isize,
    DEACTIVATE  = LifecycleGoal::EV_DEACTIVATE as isize,
    SHUTDOWN    = LifecycleGoal::EV_SHUTDOWN as isize,
    ERROR       = LifecycleGoal::EV_ERROR as isize,
}

impl Transition {
    // TODO(lucasw) impl fmt::Display instead?
    fn name(&self) -> &str {
        match *self {
            Transition::CONFIGURE => "CONFIGURE",
            Transition::CLEANUP => "CLEANUP",
            Transition::ACTIVATE => "ACTIVATE",
            Transition::DEACTIVATE => "DEACTIVATE",
            Transition::SHUTDOWN => "SHUTDOWN",
            Transition::ERROR => "ERROR",
        }
    }
}

pub enum ResultCode {
    SUCCESS = LifecycleGoal::EV_SUCCESS as isize,
    FAILURE = LifecycleGoal::EV_FAILURE as isize,
}

// struct ManagedNode {
// }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn print_transition() {
        let transition = Transition::CLEANUP;
        println!("transition: {:?} {}", transition, transition.name());
        assert_eq!(&transition.name(), &"CLEANUP");
        assert_eq!(transition as isize, LifecycleGoal::EV_CLEANUP as isize);
    }

    #[test]
    fn make_msg() {
        let msg = Lifecycle {
            header: Header::default(),
            transition: Transition::ACTIVATE as i8,
            end_state: State::ACTIVE as i8,
            result_code: ResultCode::SUCCESS as i8,
        };

        assert_eq!(msg.end_state, State::ACTIVE as i8);
    }

    #[test]
    fn make_goal_feedback() {
        // let goal = LifecycleGoal;
        // let feedback = LifecycleFeedback;
        // let action = Lifecycle::LifecycleAction;
    }
}
