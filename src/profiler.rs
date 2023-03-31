use std::time::Instant;

pub struct Profiler {
    timer: Instant,
    minimum: u128,
    maximum: u128,
    average_count: u128,
    average_accumulator: u128,
}

impl Profiler {
    pub fn new() -> Profiler {
        Profiler {
            timer: Instant::now(),
            minimum: 99999999999,
            maximum: 0,
            average_count: 0,
            average_accumulator: 0,
        }
    }
    pub fn reset(&mut self) {
        self.minimum = 99999999999;
        self.maximum = 0;
        self.average_count = 0;
        self.average_accumulator = 0;
    }
    pub fn start(&mut self) {
        self.timer = Instant::now();
    }
    pub fn stop(&mut self) {
        let elapsed: u128 = self.timer.elapsed().as_nanos();
        self.average_count += 1;
        self.average_accumulator += elapsed;
        self.minimum = self.minimum.min(elapsed);
        self.maximum = self.maximum.max(elapsed);
    }
    pub fn average(&self) -> u128 {
        if self.average_count < 1 {
            return 0;
        }
        self.average_accumulator / self.average_count
    }
    pub fn minimum(&self) -> u128 {
        self.minimum
    }
    pub fn maximum(&self) -> u128 {
        self.maximum
    }
}
