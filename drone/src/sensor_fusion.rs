use m::Float;

use crate::ImuSample;

type F = f32;

const IMU_AXIS_MAP: [usize; 3] = [0, 1, 2];
const IMU_AXIS_SCALE: [F; 3] = [1.0, 1.0, 1.0];

struct Pid {
    // tune
    k_p: F,
    k_i: F,
    k_d: F,

    // state
    last_input: F,
    sum: F,
}

impl Pid {
    fn advance(&mut self, error: F) -> F {
        self.sum += error;
        let control = self.k_p * error + self.k_i * self.sum + self.k_d * (self.last_input - error);
        self.last_input = error;

        control
    }
}

pub struct ComplementaryFilterFusion {
    /// filter tune
    /// alpha * gyro + (1-alpha) * accel
    alpha: F,

    /// current roll, pitch and yaw estimates
    orientation: [F; 3],

    /// roll, pitch and yaw targets
    target: [F; 3],

    /// roll, pitch and yaw PID contorller
    pid: [Pid; 3],
}

impl ComplementaryFilterFusion {
    pub fn new(
        alpha: F,
        orientation: [F; 3],
        target: [F; 3],
        k_p: [F; 3],
        k_i: [F; 3],
        k_d: [F; 3],
    ) -> Self {
        Self {
            alpha,
            orientation,
            target,
            pid: [
                Pid {
                    k_p: k_p[0],
                    k_i: k_i[0],
                    k_d: k_d[0],
                    last_input: 0.0,
                    sum: 0.0,
                },
                Pid {
                    k_p: k_p[1],
                    k_i: k_i[1],
                    k_d: k_d[1],
                    last_input: 0.0,
                    sum: 0.0,
                },
                Pid {
                    k_p: k_p[2],
                    k_i: k_i[2],
                    k_d: k_d[2],
                    last_input: 0.0,
                    sum: 0.0,
                },
            ],
        }
    }
}

impl ComplementaryFilterFusion {
    pub fn set_target(&mut self, target: [F; 3]) {
        self.target = target;
    }

    pub fn orientation(&mut self) -> [F; 3] {
        self.orientation
    }

    pub fn advance(&mut self, sample: impl ImuSample, dt: F) -> [F; 3] {
        let gyro_orientation = [
            self.orientation[0] + (IMU_AXIS_SCALE[0] * sample.gyro()[IMU_AXIS_MAP[0]] * dt),
            self.orientation[1] + (IMU_AXIS_SCALE[1] * sample.gyro()[IMU_AXIS_MAP[1]] * dt),
            self.orientation[2] + (IMU_AXIS_SCALE[2] * sample.gyro()[IMU_AXIS_MAP[2]] * dt),
        ];

        let gravity = [
            IMU_AXIS_SCALE[0] * sample.accel()[IMU_AXIS_MAP[0]] * dt,
            IMU_AXIS_SCALE[1] * sample.accel()[IMU_AXIS_MAP[1]] * dt,
            IMU_AXIS_SCALE[2] * sample.accel()[IMU_AXIS_MAP[2]] * dt,
        ];

        let accel_orientation = [
            (gravity[1] / (gravity[0] * gravity[0] + gravity[2] * gravity[2]).sqrt()).atan(),
            (-gravity[0] / (gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt()).atan(),
            (gravity[2] / (gravity[0] * gravity[0] + gravity[1] * gravity[1]).sqrt()).atan(),
        ];

        self.orientation[0] =
            self.alpha * gyro_orientation[0] + (1.0 - self.alpha) * accel_orientation[0];
        self.orientation[1] =
            self.alpha * gyro_orientation[1] + (1.0 - self.alpha) * accel_orientation[1];
        self.orientation[2] =
            self.alpha * gyro_orientation[2] + (1.0 - self.alpha) * accel_orientation[2];

        [
            self.pid[0].advance(self.target[0] - self.orientation[0]),
            self.pid[1].advance(self.target[1] - self.orientation[1]),
            self.pid[2].advance(self.target[2] - self.orientation[2]),
        ]
    }
}
