import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import os

class PureBevNode(Node):
    def __init__(self):
        super().__init__('pure_bev_node')

        # --- AYARLAR ---
        self.CONFIG_FILE = "/home/yigit/ros2_humble/src/mapping/config_2k.yaml"
        
        # Çıktı Penceresi Boyutları
        self.OUTPUT_W = 600 
        self.OUTPUT_H = 800 
        self.BEV_SCALE = 4.0 

        self.bridge = CvBridge()
        
        # 1. Config Yükle
        self.load_config()

        # 2. Matrisleri Hazırla
        self.init_matrices()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Görüntü Aboneliği
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color', 
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info("PURE BEV MODE HAZIR!")

    def load_config(self):
        if not os.path.exists(self.CONFIG_FILE):
            self.get_logger().error(f"HATA: '{self.CONFIG_FILE}' bulunamadi.")
            raise FileNotFoundError
        with open(self.CONFIG_FILE, 'r') as f:
            self.config = yaml.safe_load(f)

    def init_matrices(self):
        # YAML'dan Kaynak Noktaları Çek
        src_points_list = self.config.get("src_points", [])
        src_pts = np.array(src_points_list, dtype=np.float32)
        
        # Hedef Noktaları Hesapla (Dikdörtgen)
        center_x = self.OUTPUT_W / 2
        lane_width = self.OUTPUT_W / self.BEV_SCALE 
        
        dst_pts = np.float32([
            [center_x - lane_width/2, 0],         # Sol Üst
            [center_x + lane_width/2, 0],         # Sağ Üst
            [center_x + lane_width/2, self.OUTPUT_H],  # Sağ Alt
            [center_x - lane_width/2, self.OUTPUT_H]   # Sol Alt
        ])
        
        # Perspektif Dönüşüm Matrisini Oluştur
        self.M_bev = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def image_callback(self, msg):
        try:
            # Bag'den gelen görüntüyü al
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            return
        
        h_orig, w_orig = frame.shape[:2]

        # --- TEK İŞLEM: KUŞ BAKIŞI DÖNÜŞÜMÜ ---
        bev_view = cv2.warpPerspective(frame, self.M_bev, (self.OUTPUT_W, self.OUTPUT_H), flags=cv2.INTER_LINEAR)

        # Görselleştirme: Aracı temsil eden daire ve siyah bant
        # Kaputu gizlemek için siyah bant
        cv2.rectangle(bev_view, (0, self.OUTPUT_H - 80), (self.OUTPUT_W, self.OUTPUT_H), (0,0,0), -1)
        
        # Kırmızı Daire (Araç Merkezi)
        cv2.circle(bev_view, (int(self.OUTPUT_W/2), self.OUTPUT_H), 20, (0,0,255), -1) 
        
        # Bilgi Yazısı
        cv2.putText(bev_view, f"Src: {w_orig}x{h_orig}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("PURE BIRD'S EYE VIEW", bev_view)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PureBevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()