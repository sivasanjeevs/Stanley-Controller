from math import cos, sin


class CarDescription:

    def __init__(self, overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase):

        self.overall_length = overall_length
        self.overall_width = overall_width
        self.rear_overhang = rear_overhang
        self.tyre_diameter = tyre_diameter
        self.tyre_width = tyre_width
        self.axle_track = axle_track
        self.wheelbase = wheelbase

    def plot_car(self, x, y, yaw, steer):

        half_width = self.overall_width / 2

        # Tyre distances from center along the wheelbase
        front_axle_center_x = x + self.wheelbase * cos(yaw)
        front_axle_center_y = y + self.wheelbase * sin(yaw)
        rear_axle_center_x = x
        rear_axle_center_y = y

        # Outline rectangle corners relative to rear axle center
        rear_left_x = rear_axle_center_x - self.rear_overhang * cos(yaw) - half_width * sin(yaw)
        rear_left_y = rear_axle_center_y - self.rear_overhang * sin(yaw) + half_width * cos(yaw)

        rear_right_x = rear_axle_center_x - self.rear_overhang * cos(yaw) + half_width * sin(yaw)
        rear_right_y = rear_axle_center_y - self.rear_overhang * sin(yaw) - half_width * cos(yaw)

        front_left_x = front_axle_center_x + (self.overall_length - self.rear_overhang - self.wheelbase) * cos(yaw) - half_width * sin(yaw)
        front_left_y = front_axle_center_y + (self.overall_length - self.rear_overhang - self.wheelbase) * sin(yaw) + half_width * cos(yaw)

        front_right_x = front_axle_center_x + (self.overall_length - self.rear_overhang - self.wheelbase) * cos(yaw) + half_width * sin(yaw)
        front_right_y = front_axle_center_y + (self.overall_length - self.rear_overhang - self.wheelbase) * sin(yaw) - half_width * cos(yaw)

        outline_x = [rear_left_x, rear_right_x, front_right_x, front_left_x, rear_left_x]
        outline_y = [rear_left_y, rear_right_y, front_right_y, front_left_y, rear_left_y]

        # Compute tyre corner positions
        def tyre_polygon(center_x, center_y, yaw_angle):
            half_len = self.tyre_diameter / 2
            half_wid = self.tyre_width / 2
            c = cos(yaw_angle)
            s = sin(yaw_angle)
            dx1, dy1 = half_len * c, half_len * s
            dx2, dy2 = half_wid * s, -half_wid * c
            x1, y1 = center_x - dx1 - dx2, center_y - dy1 - dy2
            x2, y2 = center_x - dx1 + dx2, center_y - dy1 + dy2
            x3, y3 = center_x + dx1 + dx2, center_y + dy1 + dy2
            x4, y4 = center_x + dx1 - dx2, center_y + dy1 - dy2
            return [x1, x2, x3, x4, x1], [y1, y2, y3, y4, y1]

        # Rear tyres (aligned with body yaw)
        rear_left_center_x = rear_axle_center_x - half_width * sin(yaw)
        rear_left_center_y = rear_axle_center_y + half_width * cos(yaw)
        rear_right_center_x = rear_axle_center_x + half_width * sin(yaw)
        rear_right_center_y = rear_axle_center_y - half_width * cos(yaw)

        rr_x, rr_y = tyre_polygon(rear_right_center_x, rear_right_center_y, yaw)
        rl_x, rl_y = tyre_polygon(rear_left_center_x, rear_left_center_y, yaw)

        # Front tyres (steered by steer angle around front axle center)
        front_left_center_x = front_axle_center_x - half_width * sin(yaw)
        front_left_center_y = front_axle_center_y + half_width * cos(yaw)
        front_right_center_x = front_axle_center_x + half_width * sin(yaw)
        front_right_center_y = front_axle_center_y - half_width * cos(yaw)

        fr_x, fr_y = tyre_polygon(front_right_center_x, front_right_center_y, yaw + steer)
        fl_x, fl_y = tyre_polygon(front_left_center_x, front_left_center_y, yaw + steer)

        return (outline_x, outline_y), (fr_x, fr_y), (rr_x, rr_y), (fl_x, fl_y), (rl_x, rl_y)


