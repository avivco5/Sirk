import pygame
import socket
import struct
import time
import threading
import cv2
import numpy as np
import sys

# --- General Settings ---
# All values in this section can be customized.
DEFAULT_TARGET_IP = "192.168.1.120"  # <--- Change the ESP32's IP address here
DEFAULT_VIDEO_STREAM_URL = "rtsp://admin:nagmash4299@192.168.1.64:554/Streaming/Channels/101"
UDP_PORT = 12345

# --- MODIFICATION: Define a fixed internal resolution for scalable UI ---
INTERNAL_WIDTH, INTERNAL_HEIGHT = 1280, 720

# Axis configuration for reading and sending from the remote
# Indices: [Left Stick Horizontal, Left Stick Vertical, Right Stick Vertical, Trim 1, Trim 2, ...]
AXES_TO_SEND_INDICES = [0, 1, 2, 3, 4, 5, 6]

# Settings for mapping values from the remote (-1 to 1) to the sent values (e.g., 1000-2000)
OUT_MIN = 180
OUT_MAX = 1820
SEND_INTERVAL = 0.05  # 20 times per second

# --- Graphics Settings ---
# These colors are used for drawing on the internal surface
BG_COLOR = (20, 20, 35)
PANEL_COLOR = (40, 40, 60)
TEXT_COLOR = (230, 230, 230)
ACCENT_COLOR = (0, 150, 255)
ERROR_COLOR = (255, 80, 80)
SUCCESS_COLOR = (80, 255, 80)


# --- UI Component Classes ---

class InputBox:
    """Class to create a text box for user input."""

    def __init__(self, x, y, w, h, font, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = ACCENT_COLOR
        self.text = text
        self.font = font
        self.txt_surface = self.font.render(text, True, TEXT_COLOR)
        self.active = False

    def handle_event(self, event):
        # The event's 'pos' is already scaled by the main loop
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = True
            else:
                self.active = False
        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                self.text += event.unicode
            self.txt_surface = self.font.render(self.text, True, TEXT_COLOR)

    def draw(self, surface):
        pygame.draw.rect(surface, self.color if self.active else TEXT_COLOR, self.rect, 2, border_radius=5)
        surface.blit(self.txt_surface, (self.rect.x + 8, self.rect.y + 7))


class Button:
    """Class to create a clickable button."""

    def __init__(self, x, y, w, h, font, text, color):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.color = color
        self.font = font
        self.txt_surface = self.font.render(text, True, (255, 255, 255))
        self.txt_rect = self.txt_surface.get_rect(center=self.rect.center)

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect, border_radius=8)
        surface.blit(self.txt_surface, self.txt_rect)

    def is_clicked(self, pos):
        # The 'pos' passed here is already scaled by the main loop
        return self.rect.collidepoint(pos)


# --- Main Application Class ---

class RCControllerGUI:
    def __init__(self):
        # --- Initialize Pygame and Resources ---
        pygame.init()
        # --- MODIFICATION: Set resizable mode and create an internal surface ---
        self.screen = pygame.display.set_mode((INTERNAL_WIDTH, INTERNAL_HEIGHT), pygame.RESIZABLE)
        self.internal_surface = pygame.Surface((INTERNAL_WIDTH, INTERNAL_HEIGHT))

        pygame.display.set_caption("RC Controller Interface")
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.SysFont("calibri", 20)
        self.font_large = pygame.font.SysFont("calibri", 24, bold=True)

        # --- Initialize UI Components (positions are for the internal surface) ---
        # UDP Controls
        self.ip_input_box = InputBox(160, 20, 200, 32, self.font_small, DEFAULT_TARGET_IP)
        self.connect_button = Button(370, 20, 100, 32, self.font_small, "Connect", (0, 128, 0))
        self.disconnect_button = Button(480, 20, 110, 32, self.font_small, "Disconnect", (128, 0, 0))

        # Video Controls
        video_controls_y = INTERNAL_HEIGHT - 55
        self.stream_input_box = InputBox(570, video_controls_y, 300, 32, self.font_small, DEFAULT_VIDEO_STREAM_URL)
        self.stream_button = Button(self.stream_input_box.rect.right + 10, video_controls_y, 100, 32, self.font_small,
                                    "Stream", ACCENT_COLOR)
        self.stop_stream_button = Button(self.stream_button.rect.right + 10, video_controls_y, 120, 32, self.font_small,
                                         "Stop Stream", (200, 80, 0))

        self.input_boxes = [self.ip_input_box, self.stream_input_box]
        self.status_message = "Status: Waiting for joystick..."

        # --- Initialize State and Control Variables ---
        self.joystick = None
        self.is_joystick_connected = False
        self.axis_values = []
        self.running = True

        # UDP
        self.udp_thread = None
        self.is_udp_connected = False
        self.udp_lock = threading.Lock()
        self.sock = None

        # Video Stream
        self.video_thread = None
        self.is_streaming = False
        self.video_lock = threading.Lock()
        self.latest_frame = None

        self._initialize_joystick()

    def _initialize_joystick(self):
        if pygame.joystick.get_count() == 0:
            self.status_message = "Error: No joystick detected!"
            self.is_joystick_connected = False
            self.joystick = None
            self.axis_values = [0.0] * (max(AXES_TO_SEND_INDICES, default=-1) + 1)
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.is_joystick_connected = True

        num_axes = self.joystick.get_numaxes()
        self.axis_values = [0.0] * num_axes

        required_axes = max(AXES_TO_SEND_INDICES, default=-1) + 1
        if num_axes < required_axes:
            self.status_message = f"Warning: Joystick has {num_axes} axes, but {required_axes} are needed."
        else:
            self.status_message = f"Joystick '{self.joystick.get_name()}' connected."

    def _map_range(self, value, in_min, in_max, out_min, out_max):
        normalized_value = (value - in_min) / (in_max - in_min)
        return int(normalized_value * (out_max - out_min) + out_min)

    def _udp_sender_worker(self):
        target_ip = self.ip_input_box.text
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.status_message = f"Connecting to {target_ip}..."
            self.sock.sendto(b'', (target_ip, UDP_PORT))
        except socket.gaierror:
            self.status_message = f"Error: Hostname '{target_ip}' could not be resolved."
            self.is_udp_connected = False
            return
        except Exception as e:
            self.status_message = f"Error creating socket: {e}"
            self.is_udp_connected = False
            return

        self.status_message = f"UDP Connected to {target_ip}:{UDP_PORT}"

        while self.is_udp_connected:
            with self.udp_lock:
                current_axis_values = list(self.axis_values)
            mapped_values = []
            for axis_index in AXES_TO_SEND_INDICES:
                if axis_index < len(current_axis_values):
                    val = current_axis_values[axis_index]
                    mapped_values.append(self._map_range(val, -1.0, 1.0, OUT_MIN, OUT_MAX))
                else:
                    mapped_values.append(self._map_range(0, -1.0, 1.0, OUT_MIN, OUT_MAX))
            try:
                format_string = '<' + 'h' * len(mapped_values)
                packed_data = struct.pack(format_string, *mapped_values)
                self.sock.sendto(packed_data, (target_ip, UDP_PORT))
            except socket.error as e:
                self.status_message = f"UDP Send Error: {e}"
                time.sleep(1)
            time.sleep(SEND_INTERVAL)

        if self.sock:
            self.sock.close()
        self.status_message = "UDP Disconnected."

    def _video_stream_worker(self):
        stream_url = self.stream_input_box.text
        cap = cv2.VideoCapture(stream_url)
        if not cap.isOpened():
            print(f"Error: Could not open video stream at {stream_url}")
            self.is_streaming = False
            return
        while self.is_streaming:
            ret, frame = cap.read()
            if not ret:
                break
            with self.video_lock:
                self.latest_frame = frame
            time.sleep(1 / 60)
        cap.release()
        with self.video_lock:
            self.latest_frame = None
        print("Video stream stopped.")

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            # --- MODIFICATION: Handle window resizing ---
            elif event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)

            # --- MODIFICATION: Scale mouse coordinates for UI interaction ---
            scaled_mouse_pos = event.pos if hasattr(event, 'pos') else (0, 0)
            if hasattr(event, 'pos'):
                current_w, current_h = self.screen.get_size()
                internal_w, internal_h = self.internal_surface.get_size()
                if current_w > 0 and current_h > 0:  # Avoid division by zero
                    scaled_x = event.pos[0] * (internal_w / current_w)
                    scaled_y = event.pos[1] * (internal_h / current_h)
                    scaled_mouse_pos = (scaled_x, scaled_y)

            # --- MODIFICATION: Create a new event object with scaled positions ---
            # This ensures that widget's internal handle_event method gets the correct coordinates.
            modified_event = event
            if hasattr(event, 'pos'):
                modified_event = pygame.event.Event(event.type, dict(event.dict, pos=scaled_mouse_pos))

            if event.type == pygame.JOYDEVICEADDED:
                self._initialize_joystick()

            elif event.type == pygame.JOYDEVICEREMOVED:
                self.is_joystick_connected = False
                self.joystick = None
                self.status_message = "Joystick disconnected! UDP stream stopped."
                if self.is_udp_connected:
                    self.is_udp_connected = False
                    if self.udp_thread and self.udp_thread.is_alive():
                        self.udp_thread.join()

            elif event.type == pygame.JOYAXISMOTION and self.joystick:
                with self.udp_lock:
                    if event.axis < len(self.axis_values):
                        self.axis_values[event.axis] = event.value

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # --- MODIFICATION: Use scaled mouse position for click detection ---
                if self.connect_button.is_clicked(scaled_mouse_pos) and not self.is_udp_connected:
                    if self.is_joystick_connected:
                        self.is_udp_connected = True
                        self.udp_thread = threading.Thread(target=self._udp_sender_worker, daemon=True)
                        self.udp_thread.start()
                    else:
                        self.status_message = "Cannot connect: Joystick is disconnected."

                elif self.disconnect_button.is_clicked(scaled_mouse_pos) and self.is_udp_connected:
                    self.is_udp_connected = False
                    if self.udp_thread and self.udp_thread.is_alive():
                        self.udp_thread.join()

                elif self.stream_button.is_clicked(scaled_mouse_pos) and not self.is_streaming:
                    self.is_streaming = True
                    self.video_thread = threading.Thread(target=self._video_stream_worker, daemon=True)
                    self.video_thread.start()

                elif self.stop_stream_button.is_clicked(scaled_mouse_pos) and self.is_streaming:
                    self.is_streaming = False
                    print("Stop Stream button clicked. Signaled video thread to stop.")

            # Pass the modified event to input boxes
            for box in self.input_boxes:
                box.handle_event(modified_event)

    def draw_joystick_status_indicator(self):
        # Draw to internal_surface, anchor to the right side of the internal surface
        indicator_rect = pygame.Rect(INTERNAL_WIDTH - 220, 20, 210, 32)
        if self.is_joystick_connected:
            color, text = SUCCESS_COLOR, "Joystick: Connected"
        else:
            color, text = ERROR_COLOR, "Joystick: Disconnected"
        pygame.draw.rect(self.internal_surface, color, indicator_rect, border_radius=8)
        text_surface = self.font_small.render(text, True, (0, 0, 0))
        text_rect = text_surface.get_rect(center=indicator_rect.center)
        self.internal_surface.blit(text_surface, text_rect)

    def draw_controls(self, panel_rect):
        title = self.font_large.render("Stick & Trim Status", True, TEXT_COLOR)
        self.internal_surface.blit(title, (panel_rect.x + 20, panel_rect.y + 15))

        stick_area = pygame.Rect(panel_rect.x + 40, panel_rect.y + 60, 200, 200)
        pygame.draw.rect(self.internal_surface, BG_COLOR, stick_area, border_radius=5)
        pygame.draw.line(self.internal_surface, TEXT_COLOR, (stick_area.centerx, stick_area.top),
                         (stick_area.centerx, stick_area.bottom), 1)
        pygame.draw.line(self.internal_surface, TEXT_COLOR, (stick_area.left, stick_area.centery),
                         (stick_area.right, stick_area.centery), 1)

        stick_x_val = self.axis_values[0] if len(self.axis_values) > 0 else 0.0
        stick_y_val = self.axis_values[1] if len(self.axis_values) > 1 else 0.0
        stick_x = stick_area.centerx + stick_x_val * (stick_area.width / 2 - 10)
        stick_y = stick_area.centery + stick_y_val * (stick_area.height / 2 - 10)
        pygame.draw.circle(self.internal_surface, ACCENT_COLOR, (stick_x, stick_y), 10)

        throttle_area = pygame.Rect(panel_rect.x + 300, panel_rect.y + 60, 50, 200)
        pygame.draw.rect(self.internal_surface, BG_COLOR, throttle_area, border_radius=5)
        throttle_val = self.axis_values[2] if len(self.axis_values) > 2 else 0.0
        throttle_norm = (throttle_val + 1) / 2
        throttle_y = throttle_area.bottom - throttle_norm * throttle_area.height
        pygame.draw.rect(self.internal_surface, ACCENT_COLOR,
                         (throttle_area.x, throttle_y - 5, throttle_area.width, 10))

        trim_indices = [3, 4, 6, 7]
        for i, axis_idx in enumerate(trim_indices):
            trim_y = panel_rect.y + 300 + i * 50
            trim_area = pygame.Rect(panel_rect.x + 40, trim_y, panel_rect.width - 80, 20)
            label = self.font_small.render(f"Trim {i + 1}", True, TEXT_COLOR)
            self.internal_surface.blit(label, (trim_area.x + trim_area.width + 10, trim_area.y))
            pygame.draw.rect(self.internal_surface, BG_COLOR, trim_area, border_radius=5)
            trim_val = self.axis_values[axis_idx] if axis_idx < len(self.axis_values) else 0.0
            trim_norm = (trim_val + 1) / 2
            indicator_x = trim_area.x + trim_norm * trim_area.width
            pygame.draw.line(self.internal_surface, ACCENT_COLOR, (indicator_x, trim_area.top),
                             (indicator_x, trim_area.bottom), 5)
            pygame.draw.line(self.internal_surface, TEXT_COLOR, (trim_area.centerx, trim_area.top),
                             (trim_area.centerx, trim_area.bottom), 1)

    def draw_video_stream(self, panel_rect):
        with self.video_lock:
            frame = self.latest_frame

        video_display_rect = pygame.Rect(panel_rect.x, panel_rect.y, panel_rect.width, panel_rect.height - 70)

        if frame is not None:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_surface = pygame.surfarray.make_surface(frame_rgb.transpose([1, 0, 2]))
                scaled_surface = pygame.transform.scale(frame_surface, video_display_rect.size)
                self.internal_surface.blit(scaled_surface, video_display_rect.topleft)
            except Exception as e:
                print(f"Could not display frame: {e}")
                self.draw_placeholder(video_display_rect, "Error displaying video")
        else:
            message = "Streaming..." if self.is_streaming else "Video stream is off"
            self.draw_placeholder(video_display_rect, message)

        self.internal_surface.blit(self.font_small.render("Stream URL:", True, TEXT_COLOR),
                                   (self.stream_input_box.rect.x - 105, self.stream_input_box.rect.y + 7))
        self.stream_input_box.draw(self.internal_surface)
        self.stream_button.draw(self.internal_surface)
        self.stop_stream_button.draw(self.internal_surface)

    def draw_placeholder(self, rect, text):
        text_surface = self.font_large.render(text, True, TEXT_COLOR)
        text_rect = text_surface.get_rect(center=rect.center)
        self.internal_surface.blit(text_surface, text_rect)

    def draw(self):
        # --- MODIFICATION: All drawing now happens on self.internal_surface ---
        self.internal_surface.fill(BG_COLOR)

        # Define panel layout based on the fixed internal resolution
        left_panel_width = 420
        gap_between_panels = 20
        right_panel_x_start = 10 + left_panel_width + gap_between_panels
        right_panel_width = INTERNAL_WIDTH - right_panel_x_start - 10

        left_panel_rect = pygame.Rect(10, 70, left_panel_width, INTERNAL_HEIGHT - 80)
        right_panel_rect = pygame.Rect(right_panel_x_start, 70, right_panel_width, INTERNAL_HEIGHT - 80)

        pygame.draw.rect(self.internal_surface, PANEL_COLOR, left_panel_rect, border_radius=10)
        pygame.draw.rect(self.internal_surface, PANEL_COLOR, right_panel_rect, border_radius=10)

        # Draw all components onto the internal surface
        self.ip_input_box.draw(self.internal_surface)
        self.connect_button.draw(self.internal_surface)
        self.disconnect_button.draw(self.internal_surface)
        self.draw_joystick_status_indicator()
        self.internal_surface.blit(self.font_small.render("ESP32 IP:", True, TEXT_COLOR), (80, 27))

        self.draw_controls(left_panel_rect)
        self.draw_video_stream(right_panel_rect)

        status_color = SUCCESS_COLOR if "Connected" in self.status_message else (
            ERROR_COLOR if "Error" in self.status_message or "disconnected" in self.status_message.lower() else TEXT_COLOR)
        status_surface = self.font_small.render(self.status_message, True, status_color)
        self.internal_surface.blit(status_surface, (15, INTERNAL_HEIGHT - 30))

        # --- MODIFICATION: Scale the internal surface to the actual screen size and display it ---
        self.screen.blit(pygame.transform.scale(self.internal_surface, self.screen.get_rect().size), (0, 0))
        pygame.display.flip()

    def run(self):
        while self.running:
            self.handle_events()
            self.draw()
            self.clock.tick(60)
        self.cleanup()

    def cleanup(self):
        self.is_udp_connected = False
        self.is_streaming = False
        if self.udp_thread and self.udp_thread.is_alive():
            self.udp_thread.join()
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join()

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    app = RCControllerGUI()
    app.run()