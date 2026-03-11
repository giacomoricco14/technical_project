#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisionCoordinator(Node):
    def __init__(self):
        super().__init__('vision_coordinator')
        
        self.create_subscription(Image, '/videocamera', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.publisher_ = self.create_publisher(Bool, '/start_task', 10)
        
        self.br = CvBridge()
        self.fx = self.fy = self.cx_cam = self.cy_cam = None
        
        self.stable_frames = 0
        self.last_X = 0.0
        self.last_Y = 0.0
        
        # PARAMETRI DEL PROGETTO
        self.Z_dist_metri = 0.85 # Stima della distanza telecamera-scatola
        self.distanza_max_metri = 0.25 # La scatola deve essere a max 15cm dal centro
        self.tolleranza_mov_metri = 0.003 # Tolleranza per considerarlo fermo
        self.frame_necessari = 30

        self.get_logger().info('Visione Avviata. Cerco oggetto BLU...')

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx, self.cx_cam = msg.k[0], msg.k[2]
            self.fy, self.cy_cam = msg.k[4], msg.k[5]

    def image_callback(self, data):
        if self.fx is None: return
            
        try:
            cv_image = self.br.imgmsg_to_cv2(data, 'bgr8')
        except: return
        
        # TRACCIAMENTO COLORE BLU IN HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        robot_vicino_e_fermo = False
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: # Ignora macchioline azzurre
                M = cv2.moments(c)
                if M['m00'] != 0:
                    u, v = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    
                    # MAGIA PINHOLE
                    X_metri = (u - self.cx_cam) * self.Z_dist_metri / self.fx
                    Y_metri = (v - self.cy_cam) * self.Z_dist_metri / self.fy
                    
                    dist_centro = math.sqrt(X_metri**2 + Y_metri**2)
                    spostamento = math.sqrt((X_metri - self.last_X)**2 + (Y_metri - self.last_Y)**2)
                    
                    # GRAFICA
                    cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"X:{X_metri:.2f} Y:{Y_metri:.2f}", (u-50, v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # LOGICA
                    if dist_centro < self.distanza_max_metri: # È VICINO?
                        if spostamento <= self.tolleranza_mov_metri: # È FERMO?
                            self.stable_frames += 1
                            
                            # MOSTRA LA SCRITTA SOLO DOPO 10 FRAME DI CONFERMA (Evita i falsi positivi a schermo)
                            if self.stable_frames > 10:
                                cv2.putText(cv_image, "TARGET LOCKED", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                        else:
                            self.stable_frames = 0
                        
                        robot_vicino_e_fermo = True

                    self.last_X = X_metri
                    self.last_Y = Y_metri

        if not robot_vicino_e_fermo: self.stable_frames = 0

        if self.stable_frames > self.frame_necessari:
            self.publisher_.publish(Bool(data=True))
            self.get_logger().info('>>> OGGETTO AGGANCIATO. PARTENZA! <<<')
            self.stable_frames = -100 # Pausa lunga prima di inviare un altro segnale

        cv2.imshow("Visione Telecamera", cv_image)
        cv2.imshow("Maschera Colore", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionCoordinator()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__': main()