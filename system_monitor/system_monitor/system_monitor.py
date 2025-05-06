# Copyright (c) 2024 lemon19900815@buerjia
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# 

# !/usr/bin/python3

import rclpy
from rclpy.node import Node

import psutil
import traceback
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.update)

    def update(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # CPU 监控
        cpu_stat = DiagnosticStatus()
        cpu_stat.name = "CPU Usage"
        cpu_stat.values = [
            KeyValue(key="Total (%)", value=str(psutil.cpu_percent())),
            KeyValue(key="Cores", value=str(psutil.cpu_count()))
        ]
        msg.status.append(cpu_stat)

        # 内存监控
        mem = psutil.virtual_memory()
        mem_stat = DiagnosticStatus()
        mem_stat.name = "Memory Usage"
        mem_stat.values = [
            KeyValue(key="Total (MB)", value=str(mem.total // 1024**2)),
            KeyValue(key="Used (%)", value=str(mem.percent))
        ]
        msg.status.append(mem_stat)

        # 磁盘 I/O
        disk = psutil.disk_usage('/')
        io_stat = DiagnosticStatus()
        io_stat.name = "Disk Usage"
        io_stat.values = [
            KeyValue(key="Total (GB)", value=str(disk.total // 1024**3)),
            KeyValue(key="Used (%)", value=str(disk.percent))
        ]
        msg.status.append(io_stat)

        # 网络监控
        net = psutil.net_io_counters()
        net_stat = DiagnosticStatus()
        net_stat.name = "Network"
        net_stat.values = [
            KeyValue(key="Bytes Sent", value=str(net.bytes_sent)),
            KeyValue(key="Bytes Recv", value=str(net.bytes_recv))
        ]
        msg.status.append(net_stat)

        self.pub.publish(msg)

def main():
    try:
        rclpy.init()
        node = SystemMonitor()
        rclpy.spin(node)
        rclpy.shutdown()
    except:
        traceback.print_exc()

if __name__ == '__main__':
    main()
