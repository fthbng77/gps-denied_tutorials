  GNU nano 4.8                                                                                                                                                                                                               camera_publisher_udp.py                                                                                                                                                                                                                          

# Yayıncı Soketi Oluştur
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error as e:
    print("Soket oluşturulamadı:", e)
    exit(1)

# Kamera Bağlantısı
try:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise Exception("Kamera bağlanamıyor")
except Exception as e:
    print(e)
    exit(1)

# Çözünürlüğü Düşürün
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    try:
        ret, frame = cap.read()
        if not ret:
            print("Görüntü okunamıyor")
            break

        # Flip the image horizontally
        frame = cv2.flip(frame, -1)

        # Sıkıştırma Kalitesini Ayarlayın
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        data = buffer.tobytes()
        size = len(data)

        # Başlangıç işareti gönderin
        client_socket.sendto(b"START", ('192.168.31.104', 6161)) #host-pc ip

        # 8 KB'dan küçük parçalara böl
        for i in range(0, size, 8192):
            chunk = data[i:i + 8192]
            client_socket.sendto(chunk, ('192.168.31.104', 6161))

        # Bitiş işareti gönderin
        client_socket.sendto(b"END", ('192.168.31.104', 6161))
    except Exception as e:
        print("Bir hata oluştu:", e)
        break
