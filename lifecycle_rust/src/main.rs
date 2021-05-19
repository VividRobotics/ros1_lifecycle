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
    // gh_tx: crossbeam_channel::Sender<ServerSimpleGoalHandle<LifecycleAction>>,
    gh_rx: crossbeam_channel::Receiver<ServerSimpleGoalHandle<LifecycleAction>>,
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

        let (gh_tx, gh_rx) = crossbeam_channel::unbounded();
        /* TODO(lucasw) the handler is going to need to call activate etc. in the
         * object that holds onto this lifecycle node- so have it get passed in
         * as a parameter to default?
         */
        // fn handler(gh: ServerSimpleGoalHandle<LifecycleAction>) {
        let handler = move |gh: ServerSimpleGoalHandle<LifecycleAction>| {
            println!("action handler queued");
            if let Err(err) = gh_tx.send(gh) {
                if rosrust::is_ok() {
                    eprintln!("{:?}", err);
                    // let result = LifecycleResult { end_state: lifecycle_model::State::ACTIVE as i8 };
                    // can't use gh now
                    // gh.response().send_aborted();
                }
            }
        };

        /*
            let test = lifecycle_model::ResultCode::SUCCESS as i8;

            // TODO(lucasw) provide channels into this handler that allow
            // communication with the managed node
            let feedback = LifecycleFeedback {
                intermediate_state: lifecycle_model::ResultCode::SUCCESS as i8,
                result_code: lifecycle_model::State::Activating as i8,
            };
            gh.handle().publish_feedback(feedback);

            let result = LifecycleResult { end_state: lifecycle_model::State::ACTIVE as i8 };
            gh.response().result(result).send_succeeded();
        }
        */

        LifecycleNode {
            state: lifecycle_model::State::default(),
            action_server: ActionServer::<LifecycleAction>::new_simple("~lifecycle", handler).unwrap(),
            state_pub,
            gh_rx,
        }
    }
}

#[derive(Default)]
struct ExampleNode {
    lifecycle_node: LifecycleNode,
}

impl ExampleNode
{
    fn run(&self) {
        let rate = rosrust::rate(10.0);

        while rosrust::is_ok() {
            let possible_gh = self.lifecycle_node.gh_rx.try_recv();
            if let Ok(gh) = possible_gh {
                let transition = gh .goal().transition as isize;
                println!("transition desired {}", transition);
                /*
                // TODO(lucasw) construct a Transition from the isize/i8
                let transition = lifecycle_model::Transition::from_u32(transition as u32);
                println!("transition desired {:?} {}", transition, transition.name());

                match transition {
                    lifecycle_model::Transition::CONFIGURE => {
                        let result = LifecycleResult { end_state: lifecycle_model::State::INACTIVE as i8 };
                        gh.response().result(result).send_succeeded();
                    },
                    lifecycle_model::Transition::ACTIVATE => {
                        let result = LifecycleResult { end_state: lifecycle_model::State::ACTIVE as i8 };
                        gh.response().result(result).send_succeeded();
                    },
                    _ => {
                        gh.response().send_aborted();
                    }
                }
                */
            }
        }
    }
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
    managed_node.run();
    /*
    managed_node.configure();
    managed_node.activate();
    managed_node.deactivate();
    managed_node.shutdown();
    managed_node.finalize();
    */
    // rosrust::spin();
}
