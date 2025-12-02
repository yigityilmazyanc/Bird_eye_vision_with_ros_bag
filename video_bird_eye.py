import cv2
import numpy as np
import yaml
import os
import sys

# ---------------------------------------------------
# AYARLAR
# ---------------------------------------------------
CONFIG_FILE = "/home/yigit/ros2_humble/src/mapping/config_video.yaml"

# Çıktı Boyutu (Hem işleme hem gösterme boyutu aynı olsun ki kasmasın)
OUTPUT_W = 600 
OUTPUT_H = 800 

# Geniş Açı Çarpanı (Zoom Out)
# Bu sayı ne kadar büyükse, kamera o kadar "geriden" bakar ve etrafı görür.
BEV_SCALE = 4.0 

def load_config():
    if not os.path.exists(CONFIG_FILE):
        print(f"HATA: '{CONFIG_FILE}' bulunamadi.")
        sys.exit()
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)
    return config

def main():
    # 1. Konfigürasyonu Yükle
    config = load_config()
    
    video_path = config.get("video_path", "")
    # Eğer YAML'da video yolu yoksa varsayılanı kullan (Güvenlik)
    if not video_path: 
        video_path = "/home/yigit/Desktop/zed_video_cikisi.mp4"

    if not os.path.exists(video_path):
        print(f"HATA: Video bulunamadi: {video_path}")
        return

    # 2. Videoyu Başlat
    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    if not ret:
        print("Video okunamadi.")
        return
    
    h, w = frame.shape[:2]

    # 3. Kalibrasyon Verilerini Al
    src_points_list = config.get("src_points", [])
    src_pts = np.array(src_points_list, dtype=np.float32)

    cam_matrix = np.array(config["calibration"]["camera_matrix"], dtype=np.float32)
    dist_coeffs = np.array(config["calibration"]["dist_coeffs"], dtype=np.float32)

    # -----------------------------------------------------------------
    # OPTİMİZASYON 1: Haritaları Önceden Hesapla (Loop Dışı)
    # -----------------------------------------------------------------
    # Bu işlem işlemciyi en çok yoran kısımdır. Bunu döngü dışına alarak
    # FPS'i 3-4 katına çıkarıyoruz.
    
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeffs, (w, h), 1, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(cam_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

    # -----------------------------------------------------------------
    # OPTİMİZASYON 2: Perspektif Matrisi (Geniş Açı Ayarlı)
    # -----------------------------------------------------------------
    center_x = OUTPUT_W / 2
    
    # Yolu ekranın ortasına sıkıştırarak kenarları görmeyi sağlıyoruz
    lane_width = OUTPUT_W / BEV_SCALE 
    
    dst_pts = np.float32([
        [center_x - lane_width/2, 0],         # Sol Üst
        [center_x + lane_width/2, 0],         # Sağ Üst
        [center_x + lane_width/2, OUTPUT_H],  # Sağ Alt
        [center_x - lane_width/2, OUTPUT_H]   # Sol Alt
    ])
    
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    print(f"Video isleniyor: {video_path}")
    print(f"Genis Aci Modu (Scale: {BEV_SCALE}) - Cikmak icin 'Q' basin.")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Başa sar
            continue

        # ADIM 1: Hızlı Distorsiyon Düzeltme (Remap)
        # cv2.undistort yerine bunu kullanıyoruz, çok daha hızlı.
        undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        # ADIM 2: Kuş Bakışı Dönüşümü
        # Doğrudan hedef boyuta (600x800) çeviriyoruz. Resize gerekmiyor.
        bev_frame = cv2.warpPerspective(undistorted, M, (OUTPUT_W, OUTPUT_H), flags=cv2.INTER_LINEAR)

        # (Opsiyonel) Görsellik: Araç Referans Noktası
        cv2.circle(bev_frame, (int(OUTPUT_W/2), OUTPUT_H), 20, (0,0,255), -1) 
        cv2.putText(bev_frame, "ARAC", (int(OUTPUT_W/2)-30, OUTPUT_H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # Göster
        cv2.imshow("Optimize Edilmis Kus Bakisi", bev_frame)

        # Bekleme süresini 1ms yaptık (En yüksek FPS için)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()