import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class CleanupMission(Node):
    def __init__(self):
        super().__init__('cleanup_mission')
        # RViz'den komut alma
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self.trigger_callback, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Koordinatları 0,0 (Başlangıç) noktasına göre ayarla
        self.waypoints = [
            [-1.5,  2.0], # Salon girişi
            [ 1.5,  2.0], # Mutfak girişi
            [ 1.5, -2.0], # Alt oda girişi
            [-1.5, -2.0], # Yatak odası girişi
            [ 0.0,  0.0]  # Şarj İstasyonu
        ]
        self.current_wp = 0
        self.is_cleaning = False
        self.get_logger().info('TEMİZLİK ROBOTU HAZIR. Publish Point butonuna basınca temizlik başlar/durur.')

    def trigger_callback(self, msg):
        if not self.is_cleaning:
            self.get_logger().info('--- TEMİZLİK BAŞLATILIYOR ---')
            self.is_cleaning = True
            self.send_next_goal()
        else:
            self.get_logger().warn('--- TEMİZLİK DURDURULDU! ŞARJA DÖNÜLÜYOR ---')
            self.is_cleaning = False
            self.current_wp = len(self.waypoints) - 1 # Son waypoint şarj istasyonu
            self.send_next_goal()

    def send_next_goal(self):
        if self.current_wp < len(self.waypoints):
            target = self.waypoints[self.current_wp]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = float(target[0])
            goal_msg.pose.pose.position.y = float(target[1])
            goal_msg.pose.pose.orientation.w = 1.0
            
            self.client.wait_for_server()
            self.client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('HEDEF REDDEDİLDİ! Engel var.')
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.is_cleaning:
            self.current_wp += 1
            self.send_next_goal()

def main():
    rclpy.init()
    rclpy.spin(CleanupMission())

if __name__ == '__main__':
    main()
