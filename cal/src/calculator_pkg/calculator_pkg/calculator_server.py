import rclpy
from rclpy.node import Node
from calculator_interfaces.srv import Calculator


class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')
        self.srv = self.create_service(Calculator, 'calc', self.cb)
        self.get_logger().info('Calculator service ready: /calc')

    def cb(self, request, response):
        a = request.a
        b = request.b
        op = request.op.strip()

        response.success = True
        response.message = ""

        if op == "+":
            response.result = float(a + b)
        elif op == "-":
            response.result = float(a - b)
        elif op == "*":
            response.result = float(a * b)
        elif op == "/":
            if b == 0:
                response.success = False
                response.message = "division by zero"
                response.result = 0.0
            else:
                response.result = float(a) / float(b)
        else:
            response.success = False
            response.message = f"unknown operator: {op}"
            response.result = 0.0

        return response


def main():
    rclpy.init()
    node = CalculatorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
