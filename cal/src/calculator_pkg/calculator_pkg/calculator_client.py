import rclpy
from rclpy.node import Node
from calculator_interfaces.srv import Calculator


class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')
        self.cli = self.create_client(Calculator, 'calc')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /calc service...')

    def call(self, a: int, b: int, op: str):
        req = Calculator.Request()
        req.a = a
        req.b = b
        req.op = op

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = CalculatorClient()

    a = int(input("a (int): ").strip())
    op = input("op (+,-,*,/): ").strip()
    b = int(input("b (int): ").strip())

    res = node.call(a, b, op)

    if res is None:
        print("service call failed")
    elif res.success:
        print(f"result: {res.result}")
    else:
        print(f"error: {res.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
