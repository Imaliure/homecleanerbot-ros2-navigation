import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class CleanBotManager(Node):
    def __init__(self):
        super().__init__('clean_bot_manager')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Temizlik yapılacak odaların yaklaşık koordinatları (Haritana göre ayarlandı)
        self.clean_points = [
            [2.0, 8.0],  # Sol üst oda
            [8.0, 8.0],  # Sağ üst oda (Mutfak)
            [8.0, 2.0],  # Sağ alt oda
            [2.0, 2.0],  # Sol alt oda (Yatak odası)
            [5.0, 5.0]   # Şarj İstasyonu (Bitiş)
        ]
        self.current_idx = 0
        self.get_logger().info('Temizlik Görevi Hazır. "Start" komutu bekleniyor...')

    def start_mission(self):
        if self.current_idx < len(self.clean_points):
            pt = self.clean_points[self.current_idx]
            self.send_goal(pt[0], pt[1])
        else:
            self.get_logger().info('TEMİZLİK BİTTİ. Şarja dönüldü.')

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        self.client.wait_for_server()
        self.client.send_goal_async(goal).add_done_callback(self.done_callback)

    def done_callback(self, future):
        self.current_idx += 1
        self.start_mission()

def main():
    rclpy.init()
    node = CleanBotManager()
    node.start_mission() # Bu satırı RViz butonu tetikleyecek şekilde geliştirebiliriz
    rclpy.spin(node)

if __name__ == '__main__':
    main()
