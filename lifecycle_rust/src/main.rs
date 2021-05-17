mod lifecycle_model;

use rosrust_actionlib::action_server::ServerSimpleGoalHandle;
use rosrust_actionlib::ActionServer;
use rosrust_msg::std_msgs::Header;
use rosrust_msg::lifecycle_msgs::Lifecycle;
use rosrust_msg::lifecycle_msgs::LifecycleAction;
use rosrust_msg::lifecycle_msgs::LifecycleFeedback;
use rosrust_msg::lifecycle_msgs::LifecycleGoal;
use rosrust_msg::lifecycle_msgs::LifecycleResult;

trait ManagedNode {
    fn configure(&self) {
    }

    fn activate(&self) {
    }

    fn deactivate(&self) {
    }

    fn shutdown(&self) {
    }

    fn finalize(&self) {
    }
}

struct LifecycleNode {
    state: lifecycle_model::State,
    action_server: ActionServer::<LifecycleAction>,
    state_pub: rosrust::Publisher<Lifecycle>,
}

impl Default for LifecycleNode {
    fn default() -> Self {
        let mut state_pub = rosrust::publish("~lifecycle_state", 2).unwrap();
        state_pub.set_latching(true);

        {
            let state_msg = Lifecycle {
                header: Header::default(),
                transition: lifecycle_model::Transition::CONFIGURE as i8,
                end_state: lifecycle_model::State::UNCONFIGURED as i8,
                result_code: lifecycle_model::ResultCode::SUCCESS as i8,
            };
            state_pub.send(state_msg).unwrap();
        }

        /* TODO(lucasw) the handler is going to need to call activate etc. in the
         * object that holds onto this lifecycle node- so have it get passed in
         * as a parameter to default?
         */
        fn handler(gh: ServerSimpleGoalHandle<LifecycleAction>) {
            println!("action handler");
            let test = lifecycle_model::ResultCode::SUCCESS as i8;
            let feedback = LifecycleFeedback {
                intermediate_state: lifecycle_model::ResultCode::SUCCESS as i8,
                result_code: lifecycle_model::State::Activating as i8,
            };
            gh.handle().publish_feedback(feedback);

            let result = LifecycleResult { end_state: lifecycle_model::State::ACTIVE as i8 };
            gh.response().result(result).send_succeeded();
        }

        LifecycleNode {
            state: lifecycle_model::State::default(),
            action_server: ActionServer::<LifecycleAction>::new_simple("~lifecycle", handler).unwrap(),
            state_pub,
        }
    }
}

#[derive(Default)]
struct ExampleNode {
    lifecycle_node: LifecycleNode,
}

impl ManagedNode for ExampleNode {
    fn configure(&self) {
        println!("configuring");
    }
    fn activate(&self) {
        println!("activating");
    }
    fn deactivate(&self) {
        println!("de-activating");
    }
    fn shutdown(&self) {
        println!("shutting down");
    }
    fn finalize(&self) {
        println!("finalizing");
    }
}

fn main() {
    rosrust::init("lifecycle_action");

    // TODO(lucasw) why can't latch be set in constructor, so no mut?
    /*
    let mut state_pub = rosrust::publish("~lifecycle_state", 2).unwrap();
    state_pub.set_latching(true);
    let state_pub = state_pub;

    {
        let state_msg = Lifecycle {
            header: Header::default(),
            transition: lifecycle_model::Transition::CONFIGURE as i8,
            end_state: lifecycle_model::State::UNCONFIGURED as i8,
            result_code: lifecycle_model::ResultCode::SUCCESS as i8,
        };
        state_pub.send(state_msg).unwrap();
    }

    let server = ActionServer::<LifecycleAction>::new_simple("~lifecycle", handler).unwrap();
    */

    let managed_node = ExampleNode::default();
    /*
    managed_node.configure();
    managed_node.activate();
    managed_node.deactivate();
    managed_node.shutdown();
    managed_node.finalize();
    */
    rosrust::spin();
}
