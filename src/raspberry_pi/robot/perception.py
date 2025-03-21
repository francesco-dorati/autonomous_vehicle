from typing import List, Tuple
from raspberry_pi.data_structures.states import CartPoint, Position, State
from raspberry_pi.data_structures.maps import LocalMap
import open3d as o3d
import numpy as np
import time
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.utils.utils import Utils
from raspberry_pi.config import ROBOT_CONFIG

logger = get_logger(__name__)

class VisualOdometry:
    __prev_points_3d: np.array = np.array([])
    __last_time: float = 0

    @staticmethod
    def init():
        VisualOdometry.__prev_points_3d = np.array([])
        VisualOdometry.__last_time = time.time()

    @staticmethod
    def compute(local_map: LocalMap) -> Tuple[float, Position]:
        """_summary_

        Args:
            current_points (np.array): (M, 2) array of 2D points

        Returns:
            float: fitness
            np.array: 
        """
        try:
            logger.debug(f"ICP compute")
            start_t = time.time()
            dt = (start_t-VisualOdometry.__last_time)*1000
            logger.debug(f"ICP time since last compute: {dt:.1f} ms")

            if local_map.get_size() == 0:
                logger.error("Empty scan")
                raise Exception("Empty scan in visual odometry")
            current_points = local_map.get_cartesian_points()
            curr_points_3d = np.hstack([current_points, np.zeros((current_points.shape[0], 1))])
            if VisualOdometry.__prev_points.size == 0:
                logger.debug("ICP previous points empty")
                VisualOdometry.__prev_points_3d = curr_points_3d
                VisualOdometry.__last_time = time.time()
                return 0, Position(0, 0, 0)

            pc_curr = o3d.geometry.PointCloud()
            pc_prev = o3d.geometry.PointCloud()
            pc_curr.points = o3d.utility.Vector3dVector(curr_points_3d)
            pc_prev.points = o3d.utility.Vector3dVector(VisualOdometry.__prev_points_3d)

            # ICP computation
            threshold = ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION
            trans_init = np.identity(4)
            reg_result = o3d.pipelines.registration.registration_icp(
                pc_curr, pc_prev, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
            )
            transformation = reg_result.transformation  

            # Extract the estimated translation (in x, y, z)
            delta_translation = transformation[:3, 3]
            rotation_matrix = transformation[:2, :2] 
            dx = delta_translation[0]
            dy = -delta_translation[1]
            dth = -np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # Compute the angle
            z = Position(dx, dy, dth)

            logger.info(f"ICP transformation m:\n{reg_result.transformation}")
            logger.info(f"ICP Fitness: {reg_result.fitness}")  # Percentuale di punti corrispondenti
            logger.info(f"ICP RMSE: {reg_result.inlier_rmse}")  # Errore medio quadratico
            logger.info(f"ICP Estimated translation (x,y,z): {z}")
            dt = (time.time() - start_t)*1000
            logger.info(f"ICP time: {dt:.1f} ms")
                
            return reg_result.fitness, z

        except Exception as icp_e:
            logger.error(f"ICP localization error: {icp_e}")
        

class ExtendedKalmanFilter:
    def __init__(self):
        self.x = State(0, 0, 0, 0, 0)
        
        # Initial covariance 
        self.P = np.eye(5)

        # Process noise covariance
        self.Q = np.diag([
            0.01,  # x noise
            0.01,  # y noise
            0.01,   # θ noise
            0.01,  # v noise 
            0.01   # w noise 
        ])
        
        # Measurement noise covariance – based on ICP uncertainty
        self.base_R = np.diag([20**2, 20**2, 10**2, 1.0, 1.0])
        

    def predict(self, ds_m: float, dth_rad: float, dt_s: float):
        # Odometry calculations
        theta_rad = self.x.th + dth_rad/2  
        dx = ds_m * np.cos(theta_rad)
        dy = ds_m * np.sin(theta_rad)
        v = ds_m / dt_s if dt_s > 0 else 0
        w = dth_rad / dt_s if dt_s > 0 else 0

        # Predict new state
        self.x.vect[0] += dx
        self.x.vect[1] += dy
        self.x.vect[2] += dth_rad
        self.x.vect[2] = Utils.normalize_rad(self.x.th)
        self.x.vect[3] = v  
        self.x.vect[4] = w
        
        # Compute the Jacobian of the process model with respect to state:
        F = np.eye(5)
        F[0, 2] = -dy
        F[1, 2] = dx
        F[0, 3] = np.cos(theta_rad)
        F[1, 3] = np.sin(theta_rad)
        F[2, 4] = dt_s

        # Propagate the covariance
        self.P = F @ self.P @ F.T + self.Q

        logger.debug(f"EKF Predict: state={self.x}, P={self.P}")

    def update(self, z: Position, fitness: float):
        """
        z: measurement vector from ICP [dx, dy, dtheta] (mm, mm, mrad)
        fitness: ICP fitness score in [0, 1] (higher is better)
        """
        # Scale factor for the measurement noise; for example, if fitness is 1.0, factor=1,
        # if fitness drops, factor increases.
        k = 5.0  # Tuning parameter (adjust based on experiments)
        factor = 1.0 + k * (1.0 - fitness)
        R_effective = self.base_R * factor
        
        H = np.eye(5)  # Measurement model: identity
        y = z.vect - self.x.vect  # Innovation (residual)
        S = H @ self.P @ H.T + R_effective
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x.vect = self.x.vect + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        
        logger.debug(f"EKF Update: fitness={fitness}, factor={factor}")
        logger.debug(f"Measurement z={z.vect}, state after update={self.x}, P={self.P}")
    
    def get_position(self) -> Position:
        return Position(self.x.x, self.x.y, self.x.th)




class Perception:
    ALPHA = 0.98

    # @staticmethod
    # def filter_theta(prev_th_urad, enc_th_urad, imu_th_urad) -> int:
    #     return int(Perception.ALPHA*(prev_th_urad+enc_th_urad) + (1-Perception.ALPHA)*imu_th_urad)
    
    # @staticmethod
    # def calculate_odometry(prev_odom: Position, ds_mm: float, dth_mrad: float) -> Position:
    #     logger.debug(f"received ds {ds_mm}, th_mrad: {dth_mrad}")
    #     x = prev_odom.x + ds_mm * np.cos(prev_odom.th/1000.0 + dth_mrad/2000.0)
    #     y = prev_odom.y + ds_mm * np.sin(prev_odom.th/1000.0 + dth_mrad/2000.0)
    #     th = prev_odom.th + dth_mrad
    #     th = Utils.normalize_mrad(th)
    #     return Position(x, y, th)

    @staticmethod
    def visual_odometry(estimated_pos: Position, current_points: List[CartPoint], prev_points: List[CartPoint]) -> Tuple[float, Position]:
        try:
            t = time.time()
            # Converti entrambe le scansioni in point cloud 3D (aggiungendo z=0)
            # Convert from mm to m by dividing each coordinate by 1000.
            curr_pts = np.array([[pt.x / 1000.0, pt.y / 1000.0] for pt in current_points], dtype=np.float64)
            prev_pts = np.array([[pt.x / 1000.0, pt.y / 1000.0] for pt in prev_points], dtype=np.float64)
            
            # logger.debug(f"curr {curr_pts}")
            # logger.debug(f"prev {prev_pts}")
  
            if prev_pts.size > 0 and curr_pts.size > 0:
                pts_prev_3d = np.hstack([prev_pts, np.zeros((prev_pts.shape[0], 1))])
                pts_curr_3d = np.hstack([curr_pts, np.zeros((curr_pts.shape[0], 1))])
                
                pc_prev = o3d.geometry.PointCloud()
                pc_curr = o3d.geometry.PointCloud()
                pc_prev.points = o3d.utility.Vector3dVector(pts_prev_3d)
                pc_curr.points = o3d.utility.Vector3dVector(pts_curr_3d)
                
                # Imposta una soglia per ICP (da adattare alle unità del tuo sistema)
                threshold = 0.1
                trans_init = np.array([
                    [np.cos(estimated_pos.th/1000.0), -np.sin(estimated_pos.th/1000.0), 0, estimated_pos.x/1000.0],  # mm -> m
                    [np.sin(estimated_pos.th/1000.0), np.cos(estimated_pos.th/1000.0),  0, estimated_pos.y/1000.0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ])
                reg_result = o3d.pipelines.registration.registration_icp(
                    pc_curr, pc_prev, threshold, trans_init,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500)
                )
                logger.info(f"ICP transformation m:\n{reg_result.transformation}")
                transformation = reg_result.transformation  
                logger.info(f"ICP transformation mm:\n{transformation}")
                logger.info(f"ICP Fitness: {reg_result.fitness}")  # Percentuale di punti corrispondenti
                logger.info(f"ICP RMSE: {reg_result.inlier_rmse}")  # Errore medio quadratico

                # Estrai la traslazione stimata (in x, y, z)
                delta_translation = transformation[:3, 3]*1000
                dx: int = round(delta_translation[0])
                dy: int = round(-delta_translation[1])
                # Calculate delta_theta (rotation angle)
                rotation_matrix = transformation[:2, :2]  # Get the 2x2 rotation matrix
                dth: int = round(-np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])*1000)  # Compute the angle
                dP = Position(dx, dy, dth)

                logger.info(f"ICP Estimated translation (x,y,z): {dP}")

                dt = time.time() - t
                logger.info(f"ICP time: {dt:.3f}")

                return reg_result.fitness, dP
            
            else:
                logger.warning("Scansioni ICP vuote!")

        except Exception as icp_e:
            logger.error(f"ICP localization error: {icp_e}")