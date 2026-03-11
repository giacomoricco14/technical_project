#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import os
import time
import math # Aggiunto per calcolare le rotazioni

class Nav2BackAndForthNode(Node):
    def __init__(self):
        super().__init__('nav2_back_and_forth_controller')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- PARAMETRI BASATI SULLE COORDINATE DELLA MAPPA ---
        self.home_x = -2.5   
        self.home_y = 0.0
        
        self.target_x = 1.1 
        self.target_y = 0.0
        
        self.stop_duration = 14.0
        
        self.timer_count = 0.0
        self.current_state = 'INIT'

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Nodo Nav2 Avanti/Indietro avviato. In attesa del server Nav2...')

    def send_goal(self, x, y, yaw_angle):
        """Invia le coordinate target e l'orientamento finale a Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map' 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Magia matematica: Convertiamo l'angolo in radianti nel formato Quaternione (z, w)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_angle / 2.0)
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 ha rifiutato la destinazione!')
            return

        self.get_logger().info('Nav2 ha accettato la destinazione. Robot in viaggio (Obstacle Avoidance ATTIVO!)...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status 4 significa "SUCCEEDED"
        if status == 4: 
            if self.current_state == 'MOVING_TO_TARGET':
                self.current_state = 'STOPPED_AT_TARGET'
                self.get_logger().info(f'Arrivato sotto l\'iiwa! Mi fermo per {self.stop_duration}s...')
            
            elif self.current_state == 'MOVING_TO_HOME':
                self.current_state = 'STOPPED_AT_HOME'
                self.get_logger().info(f'Tornato alla base! Mi fermo per {self.stop_duration}s...')

    def timer_callback(self):
        if self.current_state == 'INIT':
            if self.nav_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Nav2 trovato! Inizio il ciclo partendo verso il target...')
                
                # Va verso l'iiwa guardando a Est (0 radianti)
                self.send_goal(self.target_x, self.target_y, 0.0)
                self.current_state = 'MOVING_TO_TARGET'

        elif self.current_state == 'STOPPED_AT_TARGET':
            self.timer_count += 0.1
            if self.timer_count >= self.stop_duration:
                self.timer_count = 0.0
                self.get_logger().info('Ritorno alla base in retromarcia...')
                
                # Torna verso la base guardando a Est (0 radianti)
                self.send_goal(self.home_x, self.home_y, 0.0)
                self.current_state = 'MOVING_TO_HOME'

        elif self.current_state == 'STOPPED_AT_HOME':
            if self.timer_count == 0.0:
                self.do_respawn_magic()
            
            self.timer_count += 0.1
            if self.timer_count >= self.stop_duration:
                self.timer_count = 0.0
                self.get_logger().info('Riparto verso l\'iiwa...')
                
                # Ripartiamo verso l'iiwa guardando di nuovo a Est (0 radianti)
                self.send_goal(self.target_x, self.target_y, 0.0)
                self.current_state = 'MOVING_TO_TARGET'

    def do_respawn_magic(self):
        self.get_logger().info('Eliminazione vecchio cubo e Respawn del nuovo in corso...')
        
        NOME_MONDO = "default"  
        PATH_MODELLO_SDF = "/home/user/ros2_ws/src/my-final-project/bcr_bot/models/payload_box/model.sdf" 
        COORD_X = "-2.5"  
        COORD_Y = "0.0"  
        COORD_Z = "2.0"  
        
        comando_remove = (
            f"ign service -s /world/{NOME_MONDO}/remove "
            f"--reqtype ignition.msgs.Entity "
            f"--reptype ignition.msgs.Boolean "
            f"--timeout 2000 "
            f"--req 'name: \"payload_box\" type: MODEL'"
        )
        os.system(comando_remove)
        
        time.sleep(1.0)
        
        comando_spawn = (
            f"ign service -s /world/{NOME_MONDO}/create "
            f"--reqtype ignition.msgs.EntityFactory "
            f"--reptype ignition.msgs.Boolean "
            f"--timeout 2000 "
            f"--req 'sdf_filename: \"{PATH_MODELLO_SDF}\" name: \"payload_box\" pose {{ position {{ x: {COORD_X} y: {COORD_Y} z: {COORD_Z} }} orientation {{ w: 1.0 }} }}'"
        )
        os.system(comando_spawn)
        self.get_logger().info('Nuovo cubo spawnato! Pronto per un nuovo ciclo.')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2BackAndForthNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Arresto del nodo...')
        node.nav_client.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()