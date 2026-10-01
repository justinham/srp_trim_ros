import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import gtsam
from gtsam import noiseModel
import numpy as np
from collections import deque
import math

class EstimateEgoNode(Node):
    def __init__(self):
        super().__init__('estimate_ego_node1')

        self.factor_graphs = {}

        self.fused_estimates = {
            'timestamp': 0,
            'ego_vehicle': [],
            'visible_actors': []
        }

        self.ego_sub = self.create_subscription(String, 'ego_global_measures', self.ego_callback, 10)

        self.ego_id = None

        self.ego_data_received = False
        self.infra_data_received = False

        self.pub = self.create_publisher(String, 'ego_actor_states', 10)

    def create_factor_graph(self):

        graph = gtsam.NonlinearFactorGraph()

        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.1)
        isam_params.relinearizeSkip = 1
        isam = gtsam.ISAM2(isam_params)

        initial_estimate = gtsam.Values()

        timestamps = deque(maxlen=100)

        return graph, isam, initial_estimate, timestamps

    def ego_callback(self, msg):

        data = json.loads(msg.data)

        ego_vehicle = data["ego_vehicle"]
        timestamp = data.get("timestamp", self.get_clock().now().nanoseconds/1e9)
        self.fused_estimates["timestamp"] = timestamp

        if ego_vehicle != None:
            visible_actors = data["visible_actors"]
            self.ego_id = ego_vehicle["uniqueId"]

            # self.get_logger().info(f"Received in estimate_infra {data}")

            self.update_actor_graph(ego_vehicle, timestamp=timestamp)

            for actor_global in visible_actors:
                self.update_actor_graph(actor_global, timestamp=timestamp)

        self.publish_fused_estimates()

    def update_actor_graph(self, actor_data, timestamp):

        actor_id = actor_data["uniqueId"]
        actor_type = actor_data["class"]

        # creating a factor graph for the actor if not already existing
        if actor_id not in self.factor_graphs:
            graph, isam, initial_estimate, timestamps = self.create_factor_graph()
            self.factor_graphs[actor_id] = {
                'graph': graph,
                'isam': isam,
                'initial_estimate': initial_estimate,
                'timestamps': timestamps,
                'last_measurement_time': None,
                'timesteps': 0,
                'prev_heading': [],
                'result': None
            }

        actor_entry = self.factor_graphs[actor_id]
        graph = actor_entry['graph']
        isam = actor_entry['isam']
        initial_estimate = actor_entry['initial_estimate']
        timestamps = actor_entry['timestamps']
        timesteps = actor_entry['timesteps']
        result = actor_entry['result']
        prev_heading = actor_entry['prev_heading']
        prev_timestamp = actor_entry['last_measurement_time']
        if prev_timestamp is not None:
            dt = timestamp - prev_timestamp
            if dt == 0:
                dt = 0.1
        else:
            dt = 0.1 #1st measurtment

        # adding measurements from the source to the factor graph
        actor_entry = self.add_measurements_to_graph(actor_entry, actor_data, dt)
        
        timestamps.append(timestamp)
        actor_entry['last_measurement_time'] = timestamp
        actor_entry['timesteps'] += 1

        # update the factor graph using ISAM2 and optimize
        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()

        self.collect_estimated_state(actor_id, result, actor_type)

        actor_entry['result'] = result
        graph.resize(0)
        initial_estimate.clear()
        self.factor_graphs[actor_id] = actor_entry

    def collect_estimated_state(self, actor_id, result, actor_type):
        # getting the latest timestep
        time_steps = [gtsam.Symbol(key).index() for key in result.keys() if gtsam.Symbol(key).chr() == ord('x')]
        if not time_steps:
            return
        latest_time_step = max(time_steps)

        pos_x_key = gtsam.symbol('x', latest_time_step)
        pos_y_key = gtsam.symbol('y', latest_time_step)
        vel_x_key = gtsam.symbol('u', latest_time_step)
        vel_y_key = gtsam.symbol('v', latest_time_step)
        heading_key = gtsam.symbol('h', latest_time_step)

        estimated_position = [result.atDouble(pos_x_key), result.atDouble(pos_y_key)]
        estimated_velocity = [result.atDouble(vel_x_key), result.atDouble(vel_y_key)]
        estimated_heading = result.atDouble(heading_key)

        if actor_id == self.ego_id:
            self.fused_estimates["ego_vehicle"].append({
                "uniqueId": str(actor_id),
                "class": actor_type,
                "estimated_position": {"x": estimated_position[0], "y": estimated_position[1]},
                "estimated_velocity": {"v_x": estimated_velocity[0], "v_y": estimated_velocity[1]},
                "estimated_heading": np.radians(estimated_heading)
            })
        else:
            self.fused_estimates["visible_actors"].append({
                "uniqueId": str(actor_id),
                "class": actor_type,
                "estimated_position": {"x": estimated_position[0], "y": estimated_position[1]},
                "estimated_velocity": {"v_x": estimated_velocity[0], "v_y": estimated_velocity[1]},
                "estimated_heading": np.radians(estimated_heading)
            })

    def publish_fused_estimates(self):
        if not self.fused_estimates:
            return

        fused_msg = String()
        fused_msg.data = json.dumps(self.fused_estimates)

        self.pub.publish(fused_msg)
        self.get_logger().info(f"Published ego state for all actors: {fused_msg.data}")

        self.fused_estimates = {
            'timestamp': 0,
            'refPos': None,
            'ego_vehicle': [],
            'visible_actors': []
        }

    def add_measurements_to_graph(self, actor_entry, actor_data, dt):

        graph = actor_entry['graph']
        isam = actor_entry['isam']
        initial_estimate = actor_entry['initial_estimate']
        timesteps = actor_entry['timesteps']
        result = actor_entry['result']
        prev_heading = actor_entry['prev_heading']
        position = actor_data['pos']
        velocity = actor_data['vel']
        # speed = actor_data['speed']
        heading_data = actor_data['heading']
        if len(prev_heading) > 1:
            prev_heading.append(np.radians(heading))
            recent_headings = [prev_heading[-2], prev_heading[-1]]
            absolute_headings = np.unwrap(recent_headings)
            heading = absolute_headings[-1]*180/np.pi
            actor_entry['prev_heading'] = heading
        std = {
            "posStdDev_lat": position["posStdDev_lat"],
            "posStdDev_lon": position["posStdDev_lon"],
            "velStdDev_lat": velocity["velStdDev_lat"],
            "velStdDev_lon": velocity["velStdDev_lon"]
        }

        if math.isnan(velocity["velStdDev_lat"]) or velocity["velStdDev_lat"] > 0.5:
            velocity["velStdDev_lat"] = 0.5
        if math.isnan(velocity["velStdDev_lon"]) or velocity["velStdDev_lon"] > 0.5:
            velocity["velStdDev_lon"] = 0.5
        if math.isnan(heading_data["thetaStdDev"]) or heading_data["thetaStdDev"] > 0.25:
            heading_data["thetaStdDev"] = 0.25

        # noise models for position, speed, heading based on covariances
        posX_noise = noiseModel.Isotropic.Sigma(1, position["posStdDev_lat"])
        posY_noise = noiseModel.Isotropic.Sigma(1, position["posStdDev_lon"])
        velX_noise = noiseModel.Isotropic.Sigma(1, velocity["velStdDev_lat"])
        velY_noise = noiseModel.Isotropic.Sigma(1, velocity["velStdDev_lon"])
        heading_noise = noiseModel.Isotropic.Sigma(1, heading_data["thetaStdDev"])

        # adding prior factors (noisy measurements) to the factor graph
        time_step = timesteps + 1
        pos_x_key = gtsam.symbol('x', time_step)
        pos_y_key = gtsam.symbol('y', time_step)
        vel_x_key = gtsam.symbol('u', time_step)
        vel_y_key = gtsam.symbol('v', time_step)
        heading_key = gtsam.symbol('h', time_step)

        # adding the measurements as prior factors
        graph.add(gtsam.PriorFactorDouble(pos_x_key, position['lat'], posX_noise))
        graph.add(gtsam.PriorFactorDouble(pos_y_key, position['lon'], posY_noise))
        graph.add(gtsam.PriorFactorDouble(vel_x_key, velocity['lat_vel'], velX_noise))
        graph.add(gtsam.PriorFactorDouble(vel_y_key, velocity['lon_vel'], velY_noise))
        graph.add(gtsam.PriorFactorDouble(heading_key, heading_data['theta'], heading_noise))

        # initial estimates (use previous estimates or noisy measurements for first step)
        if timesteps > 0:
            prev_time_step = time_step - 1
            initial_estimate.insert(pos_x_key, result.atDouble(gtsam.symbol('x', prev_time_step)))
            initial_estimate.insert(pos_y_key, result.atDouble(gtsam.symbol('y', prev_time_step)))
            initial_estimate.insert(vel_x_key, result.atDouble(gtsam.symbol('u', prev_time_step)))
            initial_estimate.insert(vel_y_key, result.atDouble(gtsam.symbol('v', prev_time_step)))
            initial_estimate.insert(heading_key, result.atDouble(gtsam.symbol('h', prev_time_step)))

        else:
            initial_estimate.insert(pos_x_key, position['lat'])
            initial_estimate.insert(pos_y_key, position['lon'])
            initial_estimate.insert(vel_x_key, velocity['lat_vel'])
            initial_estimate.insert(vel_y_key, velocity['lon_vel'])
            initial_estimate.insert(heading_key, heading_data['theta'])

        # include motion model (between factors) if not the first timestep
        if dt is not None and timesteps > 1:
            prev_time_step = time_step - 1
            prev_pos_x_key = gtsam.symbol('x', prev_time_step)
            prev_pos_y_key = gtsam.symbol('y', prev_time_step)
            prev_vel_x_key = gtsam.symbol('u', prev_time_step)
            prev_vel_y_key = gtsam.symbol('v', prev_time_step)
            prev_heading_key = gtsam.symbol('h', prev_time_step)

            prev_prev_pos_x_key = gtsam.symbol('x', prev_time_step-1)
            prev_prev_pos_y_key = gtsam.symbol('y', prev_time_step-1)
            prev_prev_vel_x_key = gtsam.symbol('u', prev_time_step-1)
            prev_prev_vel_y_key = gtsam.symbol('v', prev_time_step-1)
            prev_prev_heading_key = gtsam.symbol('h', prev_time_step-1)

            prev_pos_x_est = result.atDouble(prev_pos_x_key)
            prev_pos_y_est = result.atDouble(prev_pos_y_key)
            prev_vel_x_est = result.atDouble(prev_vel_x_key)
            prev_vel_y_est = result.atDouble(prev_vel_y_key)
            prev_heading_est = result.atDouble(prev_heading_key)

            prev_prev_pos_x_est = result.atDouble(prev_prev_pos_x_key)
            prev_prev_pos_y_est = result.atDouble(prev_prev_pos_y_key)
            prev_prev_vel_x_est = result.atDouble(prev_prev_vel_x_key)
            prev_prev_vel_y_est = result.atDouble(prev_prev_vel_y_key)
            prev_prev_heading_est = result.atDouble(prev_prev_heading_key)

            # marginal covariance for each state
            std_dev_position_x = np.sqrt(isam.marginalCovariance(prev_pos_x_key))
            std_dev_position_y = np.sqrt(isam.marginalCovariance(prev_pos_y_key))
            std_dev_velocity_x = np.sqrt(isam.marginalCovariance(prev_vel_x_key))
            std_dev_velocity_y = np.sqrt(isam.marginalCovariance(prev_vel_y_key))
            std_dev_heading = np.sqrt(isam.marginalCovariance(prev_heading_key))

            # using these standard deviations as the process noise for the next time step
            state_transition_noise_position_x = noiseModel.Isotropic.Sigma(1, std_dev_position_x.mean())
            state_transition_noise_position_y = noiseModel.Isotropic.Sigma(1, std_dev_position_y.mean())
            state_transition_noise_velocity_x = noiseModel.Isotropic.Sigma(1, std_dev_velocity_x.mean())
            state_transition_noise_velocity_y = noiseModel.Isotropic.Sigma(1, std_dev_velocity_y.mean())
            state_transition_noise_heading = noiseModel.Isotropic.Sigma(1, std_dev_heading.mean())

            # prev_velocity_est = prev_speed_est * np.array([np.cos(prev_heading_est), np.sin(prev_heading_est)])
            # prev_prev_velocity_est = prev_prev_speed_est * np.array([np.cos(prev_prev_heading_est), np.sin(prev_prev_heading_est)])
            prev_acc_est = [(prev_vel_x_est - prev_prev_vel_x_est)/dt, (prev_vel_y_est - prev_prev_vel_y_est)/dt]
            prev_yawrate_est = (prev_heading_est - prev_prev_heading_est)/dt

            # motion model: between factors for consecutive time steps
            predicted_position = [prev_vel_x_est * dt + 0.5 * prev_acc_est[0] * dt**2,
                                  prev_vel_y_est * dt + 0.5 * prev_acc_est[1] * dt**2]

            graph.add(gtsam.BetweenFactorDouble(prev_pos_x_key, pos_x_key, predicted_position[0], state_transition_noise_position_x))
            graph.add(gtsam.BetweenFactorDouble(prev_pos_y_key, pos_y_key, predicted_position[1], state_transition_noise_position_y))
            graph.add(gtsam.BetweenFactorDouble(prev_vel_x_key, vel_x_key, prev_acc_est[0] * dt, state_transition_noise_velocity_x))
            graph.add(gtsam.BetweenFactorDouble(prev_vel_y_key, vel_y_key, prev_acc_est[1] * dt, state_transition_noise_velocity_y))
            graph.add(gtsam.BetweenFactorDouble(prev_heading_key, heading_key, 0, state_transition_noise_heading))
        return actor_entry
    
def main(args=None):
    rclpy.init(args=args)
    node = EstimateEgoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()