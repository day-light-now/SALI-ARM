"""Five-axis ``gravity_ff_state`` callback protocol tests. No ROS, no serial.

Pins the production bug: the old panel parser only knew four-axis payload
lengths (21/22/24/34/37) and silently dropped the five-axis 40/43-field
payload.  This suite drives the real ``PanelBackend._ff_callback`` through the
same stubs as ``test_five_axis_panel_backend`` and asserts:

* five-axis 40 (no float) and 43 (float) store exactly one state with exactly
  five hold target/error values;
* legacy four-axis 21/22/24/34/37 remain readable;
* wrong dimensions are stored as an explicit rejection (never silent success);
* the explicit protocol marker (layout.dim labels / key=value markers / header
  frame) is used when present and a data-only payload falls back to length;
* the original repro script scenario stores one message for each of 37/40/43.

Run:
    python verification/hardware/five_axis_20261107/test_five_axis_ff_callback.py
"""
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import test_five_axis_panel_backend as stubs  # installs the ROS stubs # noqa: E402

control_panel = stubs.control_panel

FOUR_FLOAT_LENGTHS = (24, 37)
FIVE_FLOAT_LENGTHS = (43,)


def message(length, axes, layout=None, header=None):
    return SimpleNamespace(data=ff_payload(length, axes), layout=layout, header=header)


def ff_payload(length, axes):
    """Deterministic payload with distinguishable header/vector/hold values."""
    values = [0.0] * length
    for index, value in enumerate((0.25, 1.5, 1.0, 5.0, 7.0)):
        if index < length:
            values[index] = value
    base = 5 + 4 * axes
    for index in range(5, min(base, length)):
        values[index] = float(index)
    if length in FOUR_FLOAT_LENGTHS or length in FIVE_FLOAT_LENGTHS:
        values[base] = 1.0        # gravity_float
        values[base + 1] = 3.5    # float_kd
        values[base + 2] = 7.5    # float_seconds
        cursor = base + 3
    elif length == 22:
        values[base] = 1.0        # gravity_float flag only
        cursor = base + 1
    else:
        cursor = base
    if length in (34, 37, 40, 43):
        values[cursor] = 2.0          # lock_state -> gravity_hold
        values[cursor + 1] = 0.75     # gain scale
        values[cursor + 2] = 4.25     # remaining seconds
        for offset in range(axes):
            values[cursor + 3 + offset] = 100.0 + offset
            values[cursor + 3 + axes + offset] = 200.0 + offset
        values[cursor + 3 + 2 * axes] = 3.0   # lock reason code
        values[cursor + 4 + 2 * axes] = 9.0   # lock command id
    return values


def dim(label, size):
    return SimpleNamespace(label=label, size=size, stride=0)


def cpp_layout(axes, float_enabled):
    """The exact marker the production C++ publisher attaches to layout.dim."""
    return SimpleNamespace(dim=[
        dim("slaihand_gravity_ff_state/2", axes),
        dim("float", 1 if float_enabled else 0),
    ])


def make_backend(joint_count=5):
    backend = control_panel.PanelBackend.__new__(control_panel.PanelBackend)
    backend.joint_count = joint_count
    backend._state_lock = threading.Lock()
    backend._ff = None
    backend._stored = []
    original_store = backend._store

    def counting_store(key, value):
        backend._stored.append((key, value))
        original_store(key, value)

    backend._store = counting_store
    return backend


class FiveAxisPayloads(unittest.TestCase):
    def test_five_axis_40_and_43_store_one_state_with_five_hold_values(self):
        for length in (40, 43):
            with self.subTest(length=length):
                backend = make_backend(5)
                backend._ff_callback(message(length, 5))
                self.assertEqual(len(backend._stored), 1)
                self.assertEqual(backend._stored[0][0], "ff")
                state = backend._stored[0][1]
                self.assertEqual(state["axes"], 5)
                self.assertTrue(state["accepted"])
                self.assertIsNone(state["error"])
                self.assertEqual(state["hold_target"], [100.0, 101.0, 102.0, 103.0, 104.0])
                self.assertEqual(state["hold_error"], [200.0, 201.0, 202.0, 203.0, 204.0])
                self.assertEqual(len(state["hold_target"]), 5)
                self.assertEqual(len(state["hold_error"]), 5)
                self.assertEqual(state["lock_state"], 2)
                self.assertTrue(state["gravity_hold"])
                self.assertEqual(state["alpha"], 0.25)
                self.assertEqual(state["stage"], 5)
                self.assertEqual(state["command_id"], 7)
                self.assertEqual(state["lock_command_id"], 9)
                self.assertEqual(len(state["requested"]), 5)
                self.assertEqual(len(state["encoded"]), 5)
                self.assertEqual(len(state["applied"]), 5)
                self.assertEqual(len(state["last_written"]), 5)
                view = backend._ff_view()
                self.assertTrue(view["connected"], view)
                self.assertEqual(view["axes"], 5)
                self.assertIsNone(view["error"])
                self.assertTrue(view["protocol"])
                if length == 43:
                    self.assertTrue(state["float_available"])
                    self.assertTrue(state["float_configurable"])
                    self.assertTrue(state["gravity_float"])
                    self.assertEqual(state["float_kd"], 3.5)
                    self.assertEqual(state["float_seconds"], 7.5)
                    self.assertEqual(view["protocol_source"], "length")
                else:
                    self.assertFalse(state["float_available"])
                    self.assertFalse(state["gravity_float"])

    def test_legacy_four_axis_lengths_remain_accepted(self):
        for length in (21, 22, 24, 34, 37):
            with self.subTest(length=length):
                backend = make_backend(4)
                backend._ff_callback(message(length, 4))
                self.assertEqual(len(backend._stored), 1)
                state = backend._stored[0][1]
                self.assertEqual(state["axes"], 4)
                self.assertTrue(state["accepted"])
                self.assertEqual(state["protocol"], "slaihand_gravity_ff_state/legacy")
                self.assertEqual(len(state["requested"]), 4)
                view = backend._ff_view()
                self.assertTrue(view["connected"], view)
                self.assertEqual(view["axes"], 4)
                if length in (34, 37):
                    self.assertEqual(state["hold_target"], [100.0, 101.0, 102.0, 103.0])
                    self.assertEqual(state["hold_error"], [200.0, 201.0, 202.0, 203.0])
                    self.assertTrue(state["gravity_hold"])
                else:
                    self.assertIsNone(state["hold_target"])
                    self.assertIsNone(state["hold_error"])
                if length == 22:
                    self.assertTrue(state["float_available"])
                    self.assertFalse(state["float_configurable"])
                    self.assertTrue(state["gravity_float"])
                elif length in (24, 37):
                    self.assertTrue(state["float_available"])
                    self.assertTrue(state["gravity_float"])
                else:
                    self.assertFalse(state["float_available"])

    def test_legacy_four_axis_hold_extension_stays_four_wide(self):
        backend = make_backend(5)
        backend._ff_callback(message(37, 4))
        state = backend._stored[0][1]
        self.assertEqual(state["axes"], 4)
        self.assertEqual(len(state["hold_target"]), 4)
        self.assertEqual(len(state["hold_error"]), 4)


class WrongDimensions(unittest.TestCase):
    def test_wrong_lengths_are_rejected_explicitly_not_silently(self):
        for length in (0, 20, 38, 39, 41, 42, 44, 100):
            with self.subTest(length=length):
                backend = make_backend(5)
                backend._ff_callback(message(length, 5 if length >= 25 else 4))
                self.assertEqual(len(backend._stored), 1)
                state = backend._stored[0][1]
                self.assertFalse(state["accepted"])
                self.assertEqual(state["error"], "FF_PROTOCOL_LENGTH")
                self.assertIn(str(length), state["reason"])
                self.assertEqual(state["length"], length)
                view = backend._ff_view()
                self.assertFalse(view["connected"])
                self.assertEqual(view["error"], "FF_PROTOCOL_LENGTH")
                self.assertIn("拒绝", view["reason"])

    def test_valid_payload_recovers_after_a_rejection(self):
        backend = make_backend(5)
        backend._ff_callback(message(41, 5))
        self.assertFalse(backend._ff_view()["connected"])
        backend._ff_callback(message(40, 5))
        self.assertTrue(backend._ff_view()["connected"])
        self.assertIsNone(backend._ff_view()["error"])

    def test_non_finite_and_non_numeric_payloads_are_rejected(self):
        for values in ([float("nan")] * 40, [float("inf")] * 43, ["bad"] * 40):
            with self.subTest(values=values[0]):
                backend = make_backend(5)
                backend._ff_callback(SimpleNamespace(data=values, layout=None, header=None))
                state = backend._stored[0][1]
                self.assertFalse(state["accepted"])
                self.assertEqual(state["error"], "FF_PROTOCOL_DATA")
                self.assertFalse(backend._ff_view()["connected"])


class ExplicitProtocolMarker(unittest.TestCase):
    def test_production_dim_labels_are_used(self):
        backend = make_backend(5)
        backend._ff_callback(message(40, 5, layout=cpp_layout(5, False)))
        state = backend._stored[0][1]
        self.assertTrue(state["accepted"])
        self.assertEqual(state["axes"], 5)
        self.assertEqual(state["protocol"], "slaihand_gravity_ff_state/2")
        self.assertEqual(state["protocol_source"], "layout")
        self.assertFalse(state["float_available"])
        backend._ff_callback(message(43, 5, layout=cpp_layout(5, True)))
        state = backend._stored[-1][1]
        self.assertTrue(state["accepted"])
        self.assertTrue(state["float_available"])
        self.assertEqual(state["protocol_source"], "layout")

    def test_four_axis_dim_labels_are_honoured(self):
        backend = make_backend(4)
        backend._ff_callback(message(34, 4, layout=cpp_layout(4, False)))
        state = backend._stored[0][1]
        self.assertTrue(state["accepted"])
        self.assertEqual(state["axes"], 4)
        self.assertEqual(state["protocol_source"], "layout")

    def test_layout_conflicts_are_rejected_explicitly(self):
        backend = make_backend(5)
        # Marker says four axes but the payload is a five-axis hold payload.
        backend._ff_callback(message(43, 5, layout=cpp_layout(4, True)))
        state = backend._stored[0][1]
        self.assertFalse(state["accepted"])
        self.assertIn(state["error"], ("FF_AXIS_MISMATCH", "FF_PROTOCOL_LENGTH"))
        # Unknown axis count in the marker.
        unknown = SimpleNamespace(dim=[dim("slaihand_gravity_ff_state/2", 6)])
        backend._ff_callback(message(43, 5, layout=unknown))
        self.assertEqual(backend._stored[-1][1]["error"], "FF_PROTOCOL_LAYOUT")
        # A marker claiming float but a payload without the float block.
        backend._ff_callback(message(40, 5, layout=cpp_layout(5, True)))
        self.assertEqual(backend._stored[-1][1]["error"], "FF_PROTOCOL_LENGTH")
        # Unknown schema version is not silently accepted.
        bad = SimpleNamespace(dim=[dim("slaihand_gravity_ff_state/9", 5)])
        backend._ff_callback(message(40, 5, layout=bad))
        self.assertEqual(backend._stored[-1][1]["error"], "FF_VERSION")

    def test_explicit_malformed_metadata_never_falls_back_to_length(self):
        backend = make_backend(5)
        banana = SimpleNamespace(dim=[dim("slaihand_gravity_ff_state/banana", 5),
                                      dim("float", 1)])
        backend._ff_callback(message(43, 5, layout=banana))
        state = backend._stored[-1][1]
        self.assertFalse(state["accepted"])
        self.assertEqual(state["error"], "FF_SCHEMA")
        bad_float = SimpleNamespace(dim=[dim("slaihand_gravity_ff_state/2", 5),
                                         dim("float", 2)])
        backend._ff_callback(message(43, 5, layout=bad_float))
        state = backend._stored[-1][1]
        self.assertFalse(state["accepted"])
        self.assertEqual(state["error"], "FF_PROTOCOL_LAYOUT")
        # Explicit axes four is a hard mismatch for a five-axis controller.
        backend._ff_callback(message(37, 4, layout=cpp_layout(4, True)))
        state = backend._stored[-1][1]
        self.assertFalse(state["accepted"])
        self.assertEqual(state["error"], "FF_AXIS_MISMATCH")

    def test_key_value_layout_and_header_markers_are_defensive_fallbacks(self):
        layout = SimpleNamespace(data=["schema=slaihand_gravity_ff_state",
                                       "version=2", "axes=5", "float=1"])
        backend = make_backend(5)
        backend._ff_callback(message(43, 5, layout=layout))
        state = backend._stored[0][1]
        self.assertTrue(state["accepted"])
        self.assertEqual(state["protocol_source"], "layout")
        header = SimpleNamespace(frame_id="schema=slaihand_gravity_ff_state;"
                                           "version=2;axes=5;float=0")
        backend._ff_callback(message(40, 5, header=header))
        state = backend._stored[-1][1]
        self.assertTrue(state["accepted"])
        self.assertEqual(state["protocol_source"], "layout")
        self.assertFalse(state["float_available"])


class ReproScenario(unittest.TestCase):
    def test_repro_five_axis_ff_callback_stores_all_three_lengths(self):
        # Exact scenario from tmp/j5_connect_20261107/repro_five_axis_ff_callback.py
        backend = control_panel.PanelBackend.__new__(control_panel.PanelBackend)
        stored = {}
        for length in (37, 40, 43):
            received = []
            backend._store = lambda *args: received.append(args)
            backend._ff_callback(SimpleNamespace(data=[0.0] * length))
            stored[length] = len(received)
        self.assertEqual(stored, {37: 1, 40: 1, 43: 1})

    def test_hold_target_width_matches_axis_count(self):
        for length, axes in ((37, 4), (40, 5), (43, 5)):
            with self.subTest(length=length):
                backend = make_backend(5)
                backend._ff_callback(SimpleNamespace(data=[0.0] * length))
                view = backend._ff_view()
                # A legacy four-axis payload may be retained for diagnosis but
                # must never become five-axis control connected.
                self.assertEqual(view["connected"], axes == 5)
                self.assertEqual(view["axes"], axes)
                if axes == 4:
                    self.assertTrue(view.get("diagnostic_only"))
                else:
                    self.assertEqual(len(view["hold_target"]), axes)
                    self.assertEqual(len(view["hold_error"]), axes)


class GravityShadowIntegration(unittest.TestCase):
    def test_five_axis_payload_is_sliced_for_the_four_axis_shadow(self):
        # Regression guard: the five-axis payload used to be dropped, so the
        # four-axis diagnostic shadow only ever saw applied=None.  Now that it
        # arrives, the production status path must slice it to four values or
        # GravityShadow.estimate rejects the whole estimate.
        import tempfile
        with tempfile.TemporaryDirectory(prefix="ff-shadow-") as tmp:
            stubs.SHARE["slaihand_bringup"] = str(ROOT / "src" / "slaihand_bringup")
            stubs.SHARE["slaihand_description"] = str(stubs.DESCRIPTION)
            backend = control_panel.PanelBackend(
                stubs.FakeNode(), tmp, 60, 30, "token",
                gravity_enabled=True, joint_count=5)
            names = ["slaihand_joint%d" % index for index in range(1, 6)]
            stubs.inject(backend, names, identity=stubs.runtime_identity("a" * 64))
            values = [0.0] * 43
            values[2] = 1.0    # active feed-forward
            values[3] = 6.0    # tracking stage
            backend._ff_callback(SimpleNamespace(data=values, layout=None, header=None))
            self.assertIsNotNone(backend.gravity_shadow, backend.gravity_error)
            shadow = backend.status()["gravity_shadow"]
            driver = shadow.get("driver") or {}
            self.assertEqual(driver.get("axes"), 5)
            self.assertEqual(len(driver.get("applied")), 5)
            if shadow.get("available"):
                self.assertEqual(len(shadow["rows"]), 4)
                self.assertTrue(all(row["applied_ff_nm"] is not None
                                    for row in shadow["rows"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
