import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BagPointSelector(Node):
    def __init__(self):
        super().__init__('bag_point_selector')
        self.bridge = CvBridge()
        self.clicked_points = []
        
        # Ekran Küçültme Oranı (0.5 = Yarı Boyut)
        self.DISPLAY_SCALE = 0.5 
        
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10)
        
        print("BEKLEME: Lutfen bag dosyasini oynatin...")
        print("Goruntu gelince SPACE tusuna basip dondurun ve 4 nokta secin.")

    def click_event(self, event, x, y, flags, params):
        # params içinde orijinal 'frame' var (üzerine çizim yapmak için)
        display_frame = params 

        if event == cv2.EVENT_LBUTTONDOWN:
            # Ekranda tıklanan (küçük) koordinatı al
            small_x, small_y = x, y
            
            # Orijinal (büyük) koordinata çevir
            real_x = small_x / self.DISPLAY_SCALE
            real_y = small_y / self.DISPLAY_SCALE
            
            self.clicked_points.append([real_x, real_y])
            print(f"Secilen (Kucuk): [{small_x}, {small_y}] -> Kaydedilen (Gercek): [{int(real_x)}, {int(real_y)}]")
            
            # Görselleştirme (Küçük resim üzerine çiz)
            cv2.circle(display_frame, (small_x, small_y), 5, (0, 0, 255), -1)
            cv2.imshow("Nokta Secici (Resize Mod)", display_frame)

            if len(self.clicked_points) == 4:
                print("\n" + "="*50)
                print("--- KOPYALAMANIZ GEREKEN YENI CONFIG (Otomatik Buyutuldu) ---")
                print("src_points:")
                for pt in self.clicked_points:
                    # YAML formatında yazdıralım
                    print(f"  - [{pt[0]:.1f}, {pt[1]:.1f}]")
                print("="*50)
                print("Bu ciktiyi config_2k.yaml dosyasindaki 'src_points' kismina yapistirin.")

    def image_callback(self, msg):
        try:
            # Orijinal (Büyük) Görüntüyü Al
            original_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        # Göstermek için küçült
        h, w = original_frame.shape[:2]
        new_w = int(w * self.DISPLAY_SCALE)
        new_h = int(h * self.DISPLAY_SCALE)
        
        display_frame = cv2.resize(original_frame, (new_w, new_h))

        cv2.imshow("Nokta Secici (Resize Mod)", display_frame)
        
        key = cv2.waitKey(10)
        if key == 32: # SPACE tuşu
            print(f"\nGORUNTU DONDURULDU! (Ekran Boyutu: {new_w}x{new_h})")
            print("Lutfen sirayla tiklayin: Sol-Ust -> Sag-Ust -> Sag-Alt -> Sol-Alt")
            
            # Mouse callback'e küçültülmüş resmi gönderiyoruz
            cv2.setMouseCallback("Nokta Secici (Resize Mod)", self.click_event, display_frame)
            cv2.waitKey(0) # Sonsuza kadar bekle

def main(args=None):
    rclpy.init(args=args)
    node = BagPointSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()