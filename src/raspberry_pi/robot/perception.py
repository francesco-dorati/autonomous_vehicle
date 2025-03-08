from typing import List, Tuple
from raspberry_pi.data_structures.states import CartPoint, Position
import open3d as o3d
import numpy as np
import time
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.utils.utils import Utils

logger = get_logger(__name__)

class Perception:
    ALPHA = 0.98

    @staticmethod
    def filter_theta(prev_th_urad, enc_th_urad, imu_th_urad) -> int:
        return int(Perception.ALPHA*(prev_th_urad+enc_th_urad) + (1-Perception.ALPHA)*imu_th_urad)
    
    @staticmethod
    def calculate_odometry(prev_odom: Position, ds_mm: float, dth_mrad: float) -> Position:
        logger.debug(f"received ds {ds_mm}, th_mrad: {dth_mrad}")
        x = prev_odom.x + ds_mm * np.cos(prev_odom.th/1000.0 + dth_mrad/2000.0)
        y = prev_odom.y + ds_mm * np.sin(prev_odom.th/1000.0 + dth_mrad/2000.0)
        th = prev_odom.th + dth_mrad
        th = Utils.normalize_mrad(th)
        return Position(x, y, th)

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
                trans_init = trans_init = np.array([
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