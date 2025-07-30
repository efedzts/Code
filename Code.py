#!/usr/bin/env python3

"""
6 Motorlu Su Altı Araç Motor Test Sistemi
PyMAVLink kullanarak Pixhawk ile iletişim
Jetson Nano uyumlu
"""

from pymavlink import mavutil
import time


class UnderwaterVehicleController:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=57600):
        """
        Su altı araç kontrol sınıfı
        
        Args:
            connection_string: Pixhawk bağlantı portu
            baudrate: Baud rate (genellikle 57600 veya 115200)
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.armed = False
        self.mode = None

        # Motor kanalları (ROV/Submarine için tipik konfigürasyon)
        self.motor_channels = {
            'front_right': 1,    # Sağ ön motor
            'front_left': 2,     # Sol ön motor
            'rear_right': 3,     # Sağ arka motor
            'rear_left': 4,      # Sol arka motor
            'vertical_front': 5, # Ön dikey motor
            'vertical_rear': 6   # Arka dikey motor
        }

        # PWM değerleri (1100-1900 arası, 1500 durgun)
        self.pwm_neutral = 1500
        self.pwm_min = 1100
        self.pwm_max = 1900

    def connect(self):
        """Pixhawk'a bağlan"""
        try:
            print(f"Pixhawk'a bağlanılıyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )

            # Heartbeat bekle
            print("Heartbeat bekleniyor...")
            self.master.wait_heartbeat()
            print(f"Heartbeat alındı! System ID: {self.master.target_system}")

            # Başlangıç parametrelerini al
            self.get_vehicle_status()
            return True

        except Exception as e:
            print(f"Bağlantı hatası: {e}")
            return False

    def get_vehicle_status(self):
        """Araç durumunu kontrol et"""
        try:
            # Mod bilgisini al
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                self.mode = mavutil.mode_string_v10(msg)
                print(f"Mevcut mod: {self.mode}")

                # Armed durumunu kontrol et
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.armed = True
                    print("Araç ARMED durumda")
                else:
                    self.armed = False
                    print("Araç DISARMED durumda")

        except Exception as e:
            print(f"Durum alma hatası: {e}")

    def set_mode(self, mode):
        """Uçuş modunu değiştir"""
        try:
            # ArduSub için tipik modlar: MANUAL, STABILIZE, DEPTH_HOLD, POSITION_HOLD
            mode_id = self.master.mode_mapping().get(mode.upper())
            if mode_id is None:
                print(f"Geçersiz mod: {mode}")
                return False

            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            print(f"Mod değiştiriliyor: {mode}")
            time.sleep(1)
            self.get_vehicle_status()
            return True

        except Exception as e:
            print(f"Mod değiştirme hatası: {e}")
            return False

    def check_arm_status(self):
        """Arm durumunu kontrol et ve detaylı bilgi ver"""
        try:
            # Heartbeat al
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("✅ Araç ARMED durumda")
                    self.armed = True
                    return True
                else:
                    print("❌ Araç DISARMED durumda")
                    self.armed = False
                    return False
            else:
                print("⚠️  Heartbeat alınamadı")
                return False

        except Exception as e:
            print(f"Arm durumu kontrol hatası: {e}")
            return False

    def get_prearm_status(self):
        """Pre-arm kontrol durumlarını göster"""
        try:
            print("\n=== PRE-ARM KONTROL DURUMU ===")

            # SYS_STATUS mesajını al
            msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if msg:
                sensors_health = msg.onboard_control_sensors_health
                sensors_present = msg.onboard_control_sensors_present
                sensors_enabled = msg.onboard_control_sensors_enabled

                print(f"Sensör sağlığı: {sensors_health}")
                print(f"Mevcut sensörler: {sensors_present}")
                print(f"Aktif sensörler: {sensors_enabled}")

                # Batarya voltajı
                print(f"Batarya voltajı: {msg.voltage_battery/1000.0:.2f}V")

            # STATUSTEXT mesajlarını kontrol et (pre-arm hata mesajları için)
            print("\nPre-arm mesajları kontrol ediliyor...")
            start_time = time.time()
            while time.time() - start_time < 3:
                msg = self.master.recv_match(type='STATUSTEXT', blocking=False)
                if msg:
                    text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    if 'PreArm' in text or 'prearm' in text:
                        print(f"⚠️  Pre-arm mesajı: {text}")
                time.sleep(0.1)

        except Exception as e:
            print(f"Pre-arm durumu alma hatası: {e}")

    def arm_vehicle(self):
        """Aracı arm et"""
        try:
            print("\n=== ARM ETME İŞLEMİ ===")

            # Önce pre-arm durumunu kontrol et
            self.get_prearm_status()

            # Arm komutu gönder
            print("Arm komutu gönderiliyor...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )

            # Arm işleminin tamamlanmasını bekle
            print("Arm işlemi bekleniyor...")
            for i in range(10):  # 10 saniye bekle
                time.sleep(1)
                if self.check_arm_status():
                    print(f"✅ Araç {i+1} saniyede arm oldu!")
                    return True
                print(f"Bekleniyor... {i+1}/10")

            print("❌ Araç arm olmadı. Pre-arm kontrolleri başarısız olabilir.")

            # Arm olmama nedenlerini kontrol et
            print("\nArm olmama nedenleri kontrol ediliyor...")
            self.get_prearm_status()

            return False

        except Exception as e:
            print(f"Arm etme hatası: {e}")
            return False

    def disarm_vehicle(self):
        """Aracı disarm et"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )

            print("Araç disarm ediliyor...")
            time.sleep(2)
            self.get_vehicle_status()
            return not self.armed

        except Exception as e:
            print(f"Disarm etme hatası: {e}")
            return False

    def set_motor_pwm(self, channel, pwm_value):
        """Belirli bir motora PWM değeri gönder"""
        try:
            # PWM değerini sınırla
            pwm_value = max(self.pwm_min, min(self.pwm_max, pwm_value))

            # RC override komutu gönder
            channels = [65535] * 18  # 18 kanal, kullanılmayan kanallar için 65535
            channels[channel - 1] = pwm_value  # Kanal indeksi 0'dan başlar

            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *channels
            )
            print(f"Channel {channel} PWM {pwm_value} olarak gönderildi.")
            return True

        except Exception as e:
            print(f"PWM gönderme hatası: {e}")
            return False

    def stop_all_motors(self):
        """Tüm motorları durdur ve RC override'ı temizle"""
        print("Tüm motorlar durduruluyor...")

        # Tüm kanalları nötr konuma getir
        for channel in self.motor_channels.values():
            self.set_motor_pwm(channel, self.pwm_neutral)
        time.sleep(0.5)

        # RC override'ı tamamen temizle
        self.clear_rc_override()

    def clear_rc_override(self):
        """RC override komutlarını temizle"""
        try:
            # Tüm kanalları 65535 (ignore) yaparak override'ı temizle
            channels = [65535] * 18

            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *channels
            )

            print("RC override temizlendi")
            time.sleep(0.2)

        except Exception as e:
            print(f"RC override temizleme hatası: {e}")



def main():
    # Bağlantı bilgileri (kendi sistemine göre değiştir)
    connection_port = '/dev/ttyACM0'  # Pixhawk bağlantı portu (örnek)
    baud_rate = 57600

    rov = UnderwaterVehicleController(connection_port, baud_rate)

    if not rov.connect():
        print("Bağlantı kurulamadı, çıkılıyor.")
        return

    # Mode'u MANUAL yap (veya ihtiyaca göre DEPTH_HOLD, STABILIZE vb)
    rov.set_mode('MANUAL')

    # Aracı arm et
    if not rov.arm_vehicle():
        print("Araç arm edilemedi, program sonlandırılıyor.")
        return

    # Motor test: Tüm motorları nötrden max PWM'e sırayla çalıştır
    print("\n=== MOTOR TESTİ ===")
    for name, ch in rov.motor_channels.items():
        print(f"{name} motoru test ediliyor (Kanal {ch})")
        rov.set_motor_pwm(ch, rov.pwm_max)
        time.sleep(2)
        rov.set_motor_pwm(ch, rov.pwm_neutral)
        time.sleep(1)

    # Motorları durdur
    rov.stop_all_motors()

    # Araç disarm et
    rov.disarm_vehicle()
    print("Program tamamlandı.")


if __name__ == "__main__":
    main()
