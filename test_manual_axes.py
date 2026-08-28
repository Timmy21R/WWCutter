import importlib.util
import unittest


spec = importlib.util.spec_from_file_location("wwcutter", "WWCutterV01.py")
wwcutter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wwcutter)


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes = []

    def write(self, data):
        self.writes.append(data)


class FakeSpinBox:
    def __init__(self, value):
        self._value = value

    def value(self):
        return self._value

    def setValue(self, value):
        self._value = value


class ManualAxisTests(unittest.TestCase):
    def setUp(self):
        self.controller = wwcutter.HotWireController.__new__(
            wwcutter.HotWireController
        )
        self.controller.serial_conn = FakeSerial()
        self.controller.origin_x = FakeSpinBox(1.25)
        self.controller.origin_y = FakeSpinBox(-2.5)
        self.controller.origin_u = FakeSpinBox(3.0)
        self.controller.origin_v = FakeSpinBox(-4.0)
        self.statuses = []
        self.controller.set_status = self.statuses.append

    def test_go_to_origin_moves_all_four_axes_to_entered_coordinates(self):
        self.controller.go_to_origin()

        expected = [
            int(value * self.controller.STEPS_PER_MM)
            for value in (1.25, -2.5, 3.0, -4.0)
        ]
        self.assertEqual(
            self.controller.serial_conn.writes,
            ["MOVE,{},{},{},{}\n".format(*expected).encode()],
        )
        self.assertEqual(self.statuses, ["Moving to Origin..."])

    def test_set_current_as_origin_resets_machine_and_axis_fields(self):
        self.controller.set_current_as_origin()

        self.assertEqual(
            self.controller.serial_conn.writes,
            [b"SETPOS,0,0,0,0\n"],
        )
        self.assertEqual(
            [
                box.value()
                for box in (
                    self.controller.origin_x,
                    self.controller.origin_y,
                    self.controller.origin_u,
                    self.controller.origin_v,
                )
            ],
            [0.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(self.statuses, ["Origin set to current position"])


if __name__ == "__main__":
    unittest.main()
