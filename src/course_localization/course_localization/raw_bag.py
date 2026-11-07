"""Optional full-rate ROS recording; owns only the rosbag process it starts."""
import signal
import subprocess


class RawBag:
    def __init__(self, run_directory, topics, enabled=True):
        self.process = None
        self.log = None
        if enabled:
            self.log = (run_directory / 'rosbag.log').open('w')
            self.process = subprocess.Popen(
                ['ros2', 'bag', 'record', '--max-bag-size', '268435456', '-o', str(run_directory / 'rosbag'), *topics],
                stdout=self.log, stderr=subprocess.STDOUT,
            )

    def health(self):
        if self.process is None:
            return {'enabled': False}
        code = self.process.poll()
        return {'enabled': True, 'running': code is None, 'exit_code': code}

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
        if self.log is not None:
            self.log.close()
