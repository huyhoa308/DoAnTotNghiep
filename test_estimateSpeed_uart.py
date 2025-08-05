import numpy as np
import supervision as sv
from ultralytics import YOLO
import torch
import torch.serialization
from ultralytics.nn.tasks import DetectionModel
import cv2
from collections import defaultdict, deque
import serial
import time
from datetime import datetime

try:
    import skfuzzy as fuzz
    from skfuzzy import control as ctrl
except ImportError:
    print("Error: 'scikit-fuzzy' is not installed. Install it using 'pip install scikit-fuzzy'.")
    exit(1)

# Cấu hình cổng UART
SERIAL_PORT = "COM5"  # Thay bằng cổng thực tế
BAUD_RATE = 115200
TIMEOUT = 10  # Timeout cho UART

def log_with_timestamp(message):
    timestamp = "[" + datetime.now().strftime("%H:%M:%S.%f")[:-3] + "]"
    print(f"{timestamp} {message}")

def send_traffic_data(ser, green_time, avg_traffic_flow, avg_speed, motorcycle_count, car_count, bus_count):
    try:
        # Xóa bộ đệm trước khi gửi
        ser.flushInput()
        ser.flushOutput()

        # Tạo và gửi gói tin
        packet = f"{green_time},{avg_traffic_flow:.1f},{avg_speed:.1f},{motorcycle_count},{car_count},{bus_count}\n"
        ser.write(packet.encode())
        log_with_timestamp(f"Sent: {packet.strip()}")

        # Đọc ACK với timeout
        start_time = time.time()
        while time.time() - start_time < TIMEOUT:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                log_with_timestamp(f"Received: {response}")
                return True
            time.sleep(0.01)  # Giảm tải CPU
        log_with_timestamp("No ACK received within timeout")
        return False

    except serial.SerialException as e:
        log_with_timestamp(f"Serial error: {e}")
        return False

# Kiểm tra GPU
print("CUDA available:", torch.cuda.is_available())
print("Device count:", torch.cuda.device_count())
if torch.cuda.is_available():
    print("Device name:", torch.cuda.get_device_name(0))

# Allowlist DetectionModel
torch.serialization.add_safe_globals([DetectionModel])

# Load mô hình YOLOv8n và chuyển sang GPU nếu có
model = YOLO("C:\\Users\\Windows\\Desktop\\Ky_10\\Model_car_detection\\best.pt")
print("Model class names:", model.names)

# Ánh xạ tên class trong model sang tên chuẩn
CLASS_NAME_MAPPING = {
    "MotorCylcles": "motorcycle",
    "Car": "car",
    "Bus": "bus",
    "Truck": "truck"
}


if torch.cuda.is_available():
    model = model.to('cuda')
print("Model device:", model.device)

tracker = sv.ByteTrack()
box_annotator = sv.BoxAnnotator(thickness=1)  
label_annotator = sv.LabelAnnotator(text_scale=0.3, text_thickness=1)  

# Định nghĩa vùng quan tâm (SOURCE) và vùng đích (TARGET)
SOURCE = np.array([
    [257, 89], 
    [459, 89], 
    [-217, 392], 
    [1052, 392]
])

TARGET = np.array([
    [0, 0],
    [31, 0],
    [0, 65],
    [31, 65],
])

# Lớp ViewTransformer để chuyển đổi phối cảnh
class ViewTransformer:
    def __init__(self, source: np.ndarray, target: np.ndarray) -> None:
        source = source.astype(np.float32)
        target = target.astype(np.float32)
        self.m = cv2.getPerspectiveTransform(source, target)

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        reshaped_points = points.reshape(-1, 1, 2).astype(np.float32)
        transformed_points = cv2.perspectiveTransform(reshaped_points, self.m)
        return transformed_points.reshape(-1, 2)

# Khởi tạo ViewTransformer
view_transformer = ViewTransformer(source=SOURCE, target=TARGET)

# Lấy thông tin video
video_info = sv.VideoInfo.from_video_path("C:\\Users\\Windows\\Desktop\\Ky_10\\Model_car_detection\\Video_Test\\IMG_3470.MOV")
print(f"Original video size: {video_info.resolution_wh}, FPS: {video_info.fps}")

# Dictionary để lưu trữ tọa độ
coordinates = defaultdict(lambda: deque(maxlen=video_info.fps))

# Khởi tạo VideoCapture để đọc video
cap = cv2.VideoCapture("C:\\Users\\Windows\\Desktop\\Ky_10\\Model_car_detection\\Video_Test\\IMG_3470.MOV")

# Khởi tạo VideoWriter để ghi video với kích thước 720x405
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec H.264
out = cv2.VideoWriter(
    "C:\\Users\\Windows\\Desktop\\Ky_10\\Model_car_detection\\Video_Test\\result.mp4",
    fourcc,
    video_info.fps,
    (720, 405)  # Kích thước khớp với frame sau resize
)

# Coordinate storage and tracking for fuzzy logic
active_trackers = set()  # Track active vehicle IDs
exited_vehicles = set()  # Track vehicles that have exited to avoid double-counting
vehicle_counts_by_type = defaultdict(int)  # Đếm số lượng xe theo loại
tracker_class = {}  # Lưu class_id của mỗi tracker_id

# Biến để tích lũy dữ liệu trong 5 giây
data_window = {
    "green_times": [],
    "traffic_flows": [],
    "speeds": [],
    "vehicle_counts_by_type": defaultdict(int)
}

# Fuzzy logic setup
traffic_flow = ctrl.Antecedent(np.arange(0, 35001, 1), 'traffic_flow')
average_speed = ctrl.Antecedent(np.arange(0, 1001, 1), 'average_speed')
traffic_density = ctrl.Antecedent(np.arange(0, 301, 1), 'traffic_density')
green_duration = ctrl.Consequent(np.arange(15, 79, 1), 'green_duration')

# Fuzzy membership functions
traffic_flow['very_low'] = fuzz.trimf(traffic_flow.universe, [0, 0, 8000])
traffic_flow['low'] = fuzz.trimf(traffic_flow.universe, [8000, 11000, 14000])
traffic_flow['medium'] = fuzz.trimf(traffic_flow.universe, [14000, 17000, 20000])
traffic_flow['high'] = fuzz.trimf(traffic_flow.universe, [20000, 22500, 25000])
traffic_flow['very_high'] = fuzz.trimf(traffic_flow.universe, [25000, 30000, 35000])

average_speed['very_low'] = fuzz.trimf(average_speed.universe, [0, 0, 20])
average_speed['low'] = fuzz.trimf(average_speed.universe, [20, 30, 40])
average_speed['medium'] = fuzz.trimf(average_speed.universe, [40, 50, 60])
average_speed['high'] = fuzz.trimf(average_speed.universe, [60, 70, 80])
average_speed['very_high'] = fuzz.trimf(average_speed.universe, [80, 90, 100])

traffic_density['very_low'] = fuzz.trimf(traffic_density.universe, [0, 0, 80])
traffic_density['low'] = fuzz.trimf(traffic_density.universe, [80, 100, 120])
traffic_density['medium'] = fuzz.trimf(traffic_density.universe, [120, 140, 160])
traffic_density['high'] = fuzz.trimf(traffic_density.universe, [160, 190, 220])
traffic_density['very_high'] = fuzz.trimf(traffic_density.universe, [220, 260, 300])

green_duration['very_short'] = fuzz.trimf(green_duration.universe, [15, 20, 25])
green_duration['short'] = fuzz.trimf(green_duration.universe, [25, 30, 35])
green_duration['medium'] = fuzz.trimf(green_duration.universe, [35, 40, 45])
green_duration['long'] = fuzz.trimf(green_duration.universe, [45, 52, 60])
green_duration['very_long'] = fuzz.trimf(green_duration.universe, [60, 69, 78])

# Fuzzy rules
rules = [
    ctrl.Rule(traffic_flow['medium'] & average_speed['high'] & traffic_density['very_high'], green_duration['very_long']),
    ctrl.Rule(traffic_flow['medium'] & average_speed['high'] & traffic_density['medium'], green_duration['medium']),
    ctrl.Rule(traffic_flow['high'] & average_speed['low'] & traffic_density['very_high'], green_duration['very_long']),
    ctrl.Rule(traffic_flow['very_low'] & average_speed['very_high'] & traffic_density['very_low'], green_duration['short']),
    ctrl.Rule(traffic_flow['very_high'] & average_speed['very_low'] & traffic_density['very_high'], green_duration['very_long']),
    ctrl.Rule(traffic_flow['low'] & average_speed['medium'] & traffic_density['low'], green_duration['short']),
    ctrl.Rule(traffic_flow['high'] & average_speed['medium'] & traffic_density['high'], green_duration['long']),
    ctrl.Rule(traffic_flow['medium'] & average_speed['low'] & traffic_density['medium'], green_duration['medium']),
    ctrl.Rule(traffic_flow['low'] & average_speed['very_high'] & traffic_density['very_low'], green_duration['very_short']),
    ctrl.Rule(traffic_flow['very_high'] & average_speed['medium'] & traffic_density['high'], green_duration['very_long']),
    ctrl.Rule(traffic_flow['very_low'] & average_speed['very_low'] & traffic_density['very_low'], green_duration['very_short']),
]

# Fuzzy control system
traffic_ctrl = ctrl.ControlSystem(rules)
traffic_sim = ctrl.ControlSystemSimulation(traffic_ctrl)

def process_frame(frame: np.ndarray, frame_idx: int, vehicle_count: int, avg_speed: float, data_window: dict) -> tuple:
    # Initialize green_time with a default value
    green_time = 15  # Default green time in seconds

    # Resize frame về kích thước 720x405
    frame = cv2.resize(frame, (720, 405))
    
    results = model(frame)[0]
    detections = sv.Detections.from_ultralytics(results)
    
    # Cập nhật tracking với ByteTrack
    detections = tracker.update_with_detections(detections)

    # Lưu class_id cho từng tracker_id
    for tracker_id, class_id in zip(detections.tracker_id, detections.class_id):
        tracker_class[tracker_id] = class_id

    # Lấy tọa độ điểm neo (bottom center của bounding box)
    points = detections.get_anchors_coordinates(anchor=sv.Position.BOTTOM_CENTER)
    
    # Chuyển đổi tọa độ sang tọa độ thực tế
    if points.size > 0:
        transformed_points = view_transformer.transform_points(points=points).astype(int)
    else:
        transformed_points = np.array([])

    # Hệ số hiệu chỉnh tốc độ (có thể điều chỉnh dựa trên dữ liệu thực tế)
    calibration_factor = 1.0  # Thay đổi giá trị này nếu cần

    # Lưu trữ tọa độ và tính tốc độ
    speeds = {}
    current_trackers = set()
    for tracker_id, [x, y] in zip(detections.tracker_id, transformed_points):
        # Kiểm tra tọa độ hợp lệ trong vùng TARGET (31x65)
        if 0 <= x <= 31 and 0 <= y <= 65:
            coordinates[tracker_id].append([x, y])  # Lưu cả x và y
            current_trackers.add(tracker_id)
        
            # Tính tốc độ nếu có đủ dữ liệu
            if len(coordinates[tracker_id]) > video_info.fps // 2:
                coordinate_start = coordinates[tracker_id][-1]  # [x_start, y_start]
                coordinate_end = coordinates[tracker_id][0]     # [x_end, y_end]
                
                # Tính khoảng cách Euclidean
                distance = np.sqrt(
                    (coordinate_start[0] - coordinate_end[0])**2 +
                    (coordinate_start[1] - coordinate_end[1])**2
                )
                
                # Tính thời gian
                time = len(coordinates[tracker_id]) / video_info.fps
                
                # Tính tốc độ (km/h) với hệ số hiệu chỉnh
                speed = distance / time * 3.6 * calibration_factor
                speeds[tracker_id] = speed

    # Count vehicles exiting SOURCE for fuzzy logic
    exited_trackers = active_trackers - current_trackers
    for tracker_id in exited_trackers:
        if tracker_id not in exited_vehicles:
            vehicle_count += 1
            # Tăng bộ đếm cho loại xe tương ứng
            class_id = tracker_class.get(tracker_id, 0)  # Mặc định class_id = 0 nếu không có
            raw_class_name = model.names[class_id]
            class_name = CLASS_NAME_MAPPING.get(raw_class_name, None)
            if class_name:
                vehicle_counts_by_type[class_name] += 1
                data_window["vehicle_counts_by_type"][class_name] += 1

            exited_vehicles.add(tracker_id)
    active_trackers.clear()
    active_trackers.update(current_trackers)

    # Compute fuzzy logic inputs
    current_vehicle_count = len(detections.tracker_id)
    
    # Traffic flow (Q, vph)
    time_window = frame_idx / video_info.fps / 3600
    traffic_flow_value = vehicle_count / max(time_window, 1e-6) if frame_idx > 0 else 0

    # Average speed (K, km/h)
    current_speeds = list(speeds.values())
    avg_speed = (avg_speed * frame_idx + sum(current_speeds)) / (frame_idx + len(current_speeds)) if current_speeds else avg_speed

    # Traffic density (L, vpk)
    traffic_density_value = current_vehicle_count / 0.065 if current_vehicle_count > 0 else 0

    # Validate and clamp values
    traffic_flow_value = min(max(float(traffic_flow_value), 0), 35000)
    avg_speed = min(max(float(avg_speed), 0), 100)
    traffic_density_value = min(max(float(traffic_density_value), 0), 300)

    # Apply fuzzy logic
    inputs = [traffic_flow_value, avg_speed, traffic_density_value]
    if np.any(np.isnan(inputs)) or np.any(np.isinf(inputs)):
        print(f"Warning: Invalid fuzzy inputs at frame {frame_idx}: "
              f"Q={traffic_flow_value}, K={avg_speed}, L={traffic_density_value}")
        green_time = 15
    else:
        try:
            traffic_sim.input['traffic_flow'] = traffic_flow_value
            traffic_sim.input['average_speed'] = avg_speed
            traffic_sim.input['traffic_density'] = traffic_density_value
            traffic_sim.compute()
            green_time = traffic_sim.output.get('green_duration', 15)
        except Exception as e:
            print(f"Warning: Fuzzy computation failed at frame {frame_idx}: {str(e)}")
            green_time = 15

    # Lưu dữ liệu vào data_window
    data_window["green_times"].append(green_time)
    data_window["traffic_flows"].append(traffic_flow_value)
    data_window["speeds"].append(avg_speed)

    # Lưu tốc độ vào file CSV
    with open("speeds.csv", "a") as f:
        for tracker_id, speed in speeds.items():
            f.write(f"{tracker_id},{speed:.1f}\n")

    # Tạo danh sách nhãn (tracker ID và tốc độ)
    labels = [
        f"#{tracker_id} {speeds.get(tracker_id, 0):.1f} km/h" if tracker_id in speeds else f"#{tracker_id}"
        for tracker_id in detections.tracker_id
    ]

    # Vẽ quỹ đạo của các xe với độ dày nhỏ nhất
    for tracker_id in coordinates:
        for i in range(1, len(coordinates[tracker_id])):
            pt1 = tuple(coordinates[tracker_id][i-1])
            pt2 = tuple(coordinates[tracker_id][i])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 1)  # Độ dày 1

    # Vẽ bounding box với đường viền mỏng
    annotated_frame = box_annotator.annotate(frame.copy(), detections=detections)
    
    # Vẽ nhãn (tracker ID và tốc độ) với kích thước chữ nhỏ
    annotated_frame = label_annotator.annotate(
        annotated_frame, detections=detections, labels=labels
    )
    
    # Hiển thị các giá trị fuzzy logic và số lượng xe theo loại trên frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.3
    font_thickness = 1
    text_color = (255, 255, 255)  # White text
    bg_color = (0, 0, 0)  # Black background for readability

    # Define text lines
    text_lines = [
        f"Q: {traffic_flow_value:.1f} vph",
        f"K: {avg_speed:.1f} km/h",
        f"L: {traffic_density_value:.1f} vpk",
        f"T: {green_time:.1f} s",
    ]
    # Thêm số lượng xe theo loại
    for class_name, count in vehicle_counts_by_type.items():
        text_lines.append(f"{class_name}: {count}")

    # Calculate text position (top-left corner)
    y0, dy = 10, 15  # Starting y position and line spacing
    for i, line in enumerate(text_lines):
        y = y0 + i * dy
        # Add a semi-transparent background rectangle for readability
        text_size, _ = cv2.getTextSize(line, font, font_scale, font_thickness)
        text_w, text_h = text_size
        bg_rect = [(5, y - text_h - 2), (5 + text_w + 4, y + 2)]
        cv2.rectangle(annotated_frame, bg_rect[0], bg_rect[1], bg_color, -1)
        # Draw the text
        cv2.putText(annotated_frame, line, (5, y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)
    
    # Hiển thị frame trong cửa sổ
    cv2.imshow("Tracking", annotated_frame)
    
    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        raise KeyboardInterrupt
    
    return annotated_frame, vehicle_count, avg_speed, traffic_flow_value, traffic_density_value, green_time

try:
    # Mở cổng UART
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_with_timestamp(f"Serial port {SERIAL_PORT} opened successfully")
    except serial.SerialException as e:
        log_with_timestamp(f"Failed to open serial port: {e}")
        ser = None

    # Đọc và xử lý từng frame
    frame_idx = 0
    vehicle_count = 0
    avg_speed = 0.0
    last_uart_time = time.time()
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Xử lý frame
            annotated_frame, vehicle_count, avg_speed, traffic_flow_value, traffic_density_value, green_time = process_frame(
                frame, frame_idx, vehicle_count, avg_speed, data_window
            )
            
            # Ghi frame vào video output
            out.write(annotated_frame)
            
            # Kiểm tra thời gian để gửi dữ liệu UART mỗi 5 giây
            current_time = time.time()
            if current_time - last_uart_time >= 5 and ser is not None:
                # Tính giá trị trung bình trong cửa sổ
                avg_green_time = sum(data_window["green_times"]) / len(data_window["green_times"]) if data_window["green_times"] else 15
                avg_traffic_flow = sum(data_window["traffic_flows"]) / len(data_window["traffic_flows"]) if data_window["traffic_flows"] else 0.0
                avg_speed_window = sum(data_window["speeds"]) / len(data_window["speeds"]) if data_window["speeds"] else 0.0
                
                # Lấy số lượng xe theo loại
                motorcycle_count = data_window["vehicle_counts_by_type"].get("motorcycle", 0)
                car_count = data_window["vehicle_counts_by_type"].get("car", 0)
                bus_count = data_window["vehicle_counts_by_type"].get("bus", 0)
                
                # Gửi dữ liệu qua UART
                send_traffic_data(
                    ser,
                    avg_green_time,
                    avg_traffic_flow,
                    avg_speed_window,
                    motorcycle_count,
                    car_count,
                    bus_count
                )
                
                # Đặt lại data_window
                data_window["green_times"] = []
                data_window["traffic_flows"] = []
                data_window["speeds"] = []
                data_window["vehicle_counts_by_type"] = defaultdict(int)
                last_uart_time = current_time
            
            # In thông tin fuzzy logic và số lượng xe theo loại
            print(f"Frame {frame_idx}: Q={traffic_flow_value:.1f} vph, K={avg_speed:.1f} km/h, "
                  f"L={traffic_density_value:.1f} vpk, T={green_time:.1f} s, Vehicles: {dict(vehicle_counts_by_type)}")
            
            frame_idx += 1

    except Exception as e:
        log_with_timestamp(f"Error during frame processing: {e}")
        raise  # Re-raise the exception after logging, if needed

    finally:
        # Giải phóng tài nguyên
        cap.release()
        out.release()
        if ser is not None:
            ser.close()
            log_with_timestamp("Serial port closed")
        cv2.destroyAllWindows()

except Exception as e:
    log_with_timestamp(f"Unexpected error: {e}")
    raise  # Re-raise the exception for debugging purposes
