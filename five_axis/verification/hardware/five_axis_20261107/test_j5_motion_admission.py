"""J5 motion artifact admission + PROVISIONAL load declaration tests.

Pure Python: no ROS, no CANUSB/USB, no process/domain/page, no motion.  All
fixtures live in temporary directories; the real ``hardware_mapping.yaml`` J5
timeout stays 0 and is never modified.  The positive path uses a copy of the
mapping with J5 ``commissioning_timeout_count: 2000`` and a clearly synthetic
timeout evidence JSON that only exercises the admission parser; it is NOT real
evidence and no real timeout execution is claimed.
"""
import json
import math
import shutil
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src/slaihand_bringup/scripts"
sys.path.insert(0, str(SCRIPTS))

import j5_motion_artifact as admission  # noqa: E402
import load_declaration  # noqa: E402

MAPPING = ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_five_axis_motion_policy.yaml"
LOAD = ROOT / "src/slaihand_bringup/config/load_3p5kg_provisional.yaml"
MODEL = ROOT / "src/slaihand_description/urdf/slaihand_five_axis.urdf"
BASE_NOW = 1794100000.0
DEVICE = "/dev/ttyUSB0"


# --------------------------------------------------------------------- helpers
def sha256(path):
    return admission.sha256_file(path)


def write_mapping(directory, timeout=2000):
    """Copy the real mapping and rewrite ONLY the slaihand_joint5 timeout value."""
    text = MAPPING.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    in_j5, output = False, []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("slaihand_joint") and stripped.endswith(":"):
            in_j5 = stripped == "slaihand_joint5:"
        if in_j5 and stripped.startswith("commissioning_timeout_count:"):
            line = line[:line.index(":") + 1] + " %d\n" % timeout
        output.append(line)
    path = Path(directory) / "hardware_mapping.yaml"
    path.write_text("".join(output), encoding="utf-8")
    return path


def copy_file(source, directory, name=None):
    target = Path(directory) / (name or Path(source).name)
    shutil.copyfile(source, target)
    return target


def evidence_document(now=BASE_NOW):
    observed = datetime.fromtimestamp(now - 60, tz=timezone.utc).isoformat()
    axes = [{"axis": "J%d" % axis, "can_id": axis, "motor_state": 0,
             "feedback_age": 0.02, "valid": True,
             "feedback_payload": "05 99 ff 7f f7 ff 1b 1a"}
            for axis in range(1, 6)]
    return {
        "schema": "slaihand_j5_timeout_setup/1",
        "ok": True,
        "exit_code": 0,
        "error": None,
        "session_id": "a" * 32,
        "host_boot_id": "boot-1234",
        "tool": {"name": "j5_timeout_setup.py", "sha256": "b" * 64},
        "observed_at": observed,
        "started_at": observed,
        "completed_at": observed,
        "device": DEVICE,
        "port_argument": DEVICE,
        "baud": 921600,
        "exclusive": {"flock": True, "tiocm": True, "locked": True},
        "timeout_before": 0,
        "timeout_after": 2000,
        "j5_id": 5,
        "master_id": 0,
        "control_mode": 1,
        "profile": {"pmax_rad": 12.5, "vmax_rad_s": 10.0, "tmax_nm": 28.0},
        "all_motor_state_zero": True,
        "motor_state_all_zero": True,
        "axes": axes,
        "write_outcome": "complete",
        "write_frame_hex": "aa" * 30,
        "write_data_hex": "05 00 55 09 d0 07 00 00",
        "write_attempts": 1,
        "parameter_writes": 1,
        "readback": {"rid": 9, "expected": 2000, "observed": 2000,
                     "attempts": 2, "matched": True},
        "readback_hex": "05 00 55 09 d0 07 00 00",
        "motor_zero_written": False,
        "flash_written": False,
        "enabled": False,
        "mit_sent": False,
        "audit_path": "verification/hardware/five_axis_20261107/evidence/"
                      "j5_timeout_setup_evidence.json",
    }


def write_evidence(directory, document, name="j5_timeout_setup_evidence.json"):
    path = Path(directory) / name
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def reseal(artifact, mutate):
    """Mutate a copy of the payload and recompute the canonical SHA (forgery)."""
    document = json.loads(json.dumps(artifact))
    mutate(document["payload"])
    document["sha256"] = admission.payload_sha256(document["payload"])
    return document


def append_bytes(path, text="\n# tampered\n"):
    with open(path, "a", encoding="utf-8") as stream:
        stream.write(text)
    return path


class AdmissionBase(unittest.TestCase):
    def setUp(self):
        holder = tempfile.TemporaryDirectory(prefix="j5-admission-")
        self.addCleanup(holder.cleanup)
        self.tmp = Path(holder.name)
        self.now = BASE_NOW
        self.mapping = write_mapping(self.tmp, timeout=2000)
        self.evidence = write_evidence(self.tmp, evidence_document(self.now))

    def build(self, *, mapping=None, evidence=None, model=MODEL, policy=POLICY,
              load=LOAD, device=None, output="artifact.json", now=None):
        return admission.build_artifact(
            mapping=mapping if mapping is not None else self.mapping,
            model=model, policy=policy, load_profile=load,
            timeout_evidence=evidence if evidence is not None else self.evidence,
            device=device, now=self.now if now is None else now,
            output=(self.tmp / output) if output else None)

    def validate(self, artifact, *, mapping=None, model=MODEL, policy=POLICY,
                 load=LOAD, device=DEVICE, evidence="__default__", now=None,
                 max_age_s=admission.DEFAULT_MAX_AGE_S):
        if evidence == "__default__":
            evidence = self.evidence
        return admission.validate_artifact(
            artifact,
            mapping if mapping is not None else self.mapping,
            model, policy, load,
            expected_device=device,
            timeout_evidence=evidence,
            max_age_s=max_age_s,
            now=self.now + 60 if now is None else now)

    def assert_code(self, code, function, *args, **kwargs):
        with self.assertRaises(admission.AdmissionError) as context:
            function(*args, **kwargs)
        self.assertEqual(context.exception.code, code, str(context.exception))
        return context.exception


# --------------------------------------------------------- load declaration
class ReadOnlyTimeoutRoundTripTests(AdmissionBase):
    def setUp(self):
        super().setUp()
        document = evidence_document(self.now)
        document.update(write_outcome="verified_existing", timeout_before=2000,
                        write_attempts=0, parameter_writes=0,
                        write_data_hex=None, write_frame_hex=None)
        self.evidence = write_evidence(self.tmp, document)

    def test_read_only_metadata_survives_build_and_validation(self):
        artifact = self.build()
        timeout = self.validate(artifact)["timeout"]
        for key, expected in {"timeout_before": 2000, "timeout_after": 2000,
                              "write_outcome": "verified_existing",
                              "write_attempts": 0, "parameter_writes": 0}.items():
            self.assertEqual(timeout.get(key), expected, key)

    def test_resealed_write_claim_conflicts_with_bound_read_only_evidence(self):
        artifact = self.build()
        forged = reseal(artifact, lambda p: p["timeout"].update(
            timeout_before=0, write_outcome="complete", write_attempts=1,
            parameter_writes=1))
        self.assert_code("TIMEOUT_EVIDENCE_MISMATCH", self.validate, forged)

    def test_read_only_evidence_rejects_nonzero_write_counts(self):
        for key in ("write_attempts", "parameter_writes"):
            document = json.loads(self.evidence.read_text(encoding="utf-8"))
            document[key] = 1
            path = write_evidence(self.tmp, document, "contradiction.json")
            self.assert_code("TIMEOUT_EVIDENCE_WRITE_ATTEMPTS" if key == "write_attempts"
                             else "TIMEOUT_EVIDENCE_PARAMETER_WRITES", self.build,
                             evidence=path)

    def test_captured_gc05_evidence_roundtrip_at_historical_time(self):
        # Replay only: never write an artifact to disk or claim current admission.
        path = HERE / ('gravity_sampling_v2_20261109/execution_20261109/evidence/'
                       'j5_timeout_verify_existing_20261109_session2.json')
        document = json.loads(path.read_text(encoding="utf-8"))
        historical_now = datetime.fromisoformat(document["observed_at"]).timestamp() + 60
        artifact = self.build(evidence=path, now=historical_now)
        payload = self.validate(artifact, evidence=path, device=document["device"],
                                now=historical_now + 1)
        self.assertEqual(payload["timeout"]["timeout_before"], 2000)
        self.assertEqual(payload["timeout"]["write_outcome"], "verified_existing")
        self.assertEqual(payload["timeout"]["parameter_writes"], 0)

    def test_embedded_read_only_counts_must_agree_without_evidence_path(self):
        artifact = self.build()
        for key in ("write_attempts", "parameter_writes"):
            forged = reseal(artifact, lambda p: p["timeout"].update({key: 1}))
            self.assert_code("TIMEOUT_EVIDENCE_WRITE_ATTEMPTS" if key == "write_attempts"
                             else "TIMEOUT_EVIDENCE_PARAMETER_WRITES", self.validate,
                             forged, evidence=None)


class LoadDeclarationTests(unittest.TestCase):
    def document(self):
        return yaml.safe_load(LOAD.read_text(encoding="utf-8"))

    def write_declaration(self, mutate):
        holder = tempfile.TemporaryDirectory(prefix="j5-load-")
        self.addCleanup(holder.cleanup)
        document = self.document()
        mutate(document)
        path = Path(holder.name) / load_declaration.CONFIG_FILENAME
        path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        return path

    def reject(self, mutate, code):
        path = self.write_declaration(mutate)
        with self.assertRaises(load_declaration.LoadDeclarationError) as context:
            load_declaration.load_declaration(path)
        self.assertEqual(context.exception.code, code, str(context.exception))

    def test_provisional_declaration_binds_actual_file_sha(self):
        declaration = load_declaration.load_declaration(LOAD)
        self.assertEqual(declaration["schema"], "slaihand_load_declaration/1")
        self.assertEqual(declaration["load_id"], "j5_3p5kg_provisional")
        self.assertEqual(declaration["mass_kg"], 3.5)
        self.assertEqual(declaration["mass_source"], "user_estimate_only_not_measured")
        self.assertEqual(declaration["gravity_status"], "PROVISIONAL")
        self.assertIs(declaration["formal_gravity_allowed"], False)
        self.assertIs(declaration["float_enabled"], False)
        self.assertEqual(declaration["sha256"], sha256(LOAD))
        self.assertNotEqual(declaration["load_id"], "no_end_load")

    def test_sha_is_of_actual_bytes(self):
        source = LOAD.read_bytes()
        holder = tempfile.TemporaryDirectory(prefix="j5-load-sha-")
        self.addCleanup(holder.cleanup)
        path = Path(holder.name) / load_declaration.CONFIG_FILENAME
        path.write_bytes(source)
        same = load_declaration.load_declaration(path)
        path.write_bytes(source + b"# changed\n")
        changed = load_declaration.load_declaration(path)
        self.assertEqual(same["sha256"], load_declaration.sha256_file(LOAD))
        self.assertNotEqual(same["sha256"], changed["sha256"])

    def test_no_end_load_rejected(self):
        self.reject(lambda document: document.update(load_id="no_end_load"),
                    "LOAD_ID_STALE")

    def test_unknown_fields_must_stay_unknown(self):
        self.reject(lambda document: document.update(geometry_status=0.1),
                    "LOAD_GEOMETRY_NOT_UNKNOWN")
        self.reject(lambda document: document.update(com_status=[0.0, 0.0, 0.0]),
                    "LOAD_COM_NOT_UNKNOWN")
        self.reject(lambda document: document.update(mount_status="measured"),
                    "LOAD_MOUNT_NOT_UNKNOWN")

    def test_fabricated_measurements_and_mass_source_rejected(self):
        self.reject(lambda document: document.update(measurements=[{"mass": 3.5}]),
                    "LOAD_FABRICATED_MEASUREMENTS")
        self.reject(lambda document: document.update(mass_source="measured_on_scale"),
                    "LOAD_MASS_SOURCE_UNMEASURED")

    def test_gravity_ff_and_float_cannot_be_enabled(self):
        self.reject(lambda document: document.update(formal_gravity_allowed=True),
                    "LOAD_FORMAL_GRAVITY_ALLOWED")
        self.reject(lambda document: document.update(gravity_fit_allowed=True),
                    "LOAD_GRAVITY_FIT_ALLOWED")
        self.reject(lambda document: document.update(float_enabled=True),
                    "LOAD_FLOAT_ENABLED")
        self.reject(lambda document: document.update(gravity_ff_alpha=0.1),
                    "LOAD_FF_ALPHA_NONZERO")
        self.reject(lambda document: document.update(gravity_status="QUALIFIED"),
                    "LOAD_GRAVITY_STATUS")

    def test_schema_and_missing_file_rejected(self):
        self.reject(lambda document: document.update(schema="slaihand_load_declaration/2"),
                    "LOAD_DECLARATION_SCHEMA")
        with self.assertRaises(load_declaration.LoadDeclarationError) as context:
            load_declaration.load_declaration(ROOT / "does_not_exist.yaml")
        self.assertEqual(context.exception.code, "LOAD_DECLARATION_MISSING")


# ------------------------------------------------------------------ building
class ArtifactBuildTests(AdmissionBase):
    def test_build_binds_current_files_and_evidence(self):
        artifact = self.build()
        self.assertEqual(set(artifact), {"schema", "payload", "sha256"})
        self.assertEqual(artifact["schema"], admission.SCHEMA)
        self.assertEqual(artifact["sha256"], admission.payload_sha256(artifact["payload"]))
        payload = artifact["payload"]
        self.assertEqual(payload["qualification"], "j5_motion")
        self.assertEqual(payload["joint"], "slaihand_joint5")
        self.assertEqual(payload["load_id"], "j5_3p5kg_provisional")
        self.assertEqual(payload["load"]["gravity_status"], "PROVISIONAL")
        self.assertIs(payload["load"]["formal_gravity_allowed"], False)
        self.assertIs(payload["load"]["float_enabled"], False)
        self.assertEqual(payload["policy"]["gravity_ff_alpha"], 0.0)
        self.assertEqual(payload["hashes"]["mapping_sha256"], sha256(self.mapping))
        self.assertEqual(payload["hashes"]["model_sha256"], sha256(MODEL))
        self.assertEqual(payload["hashes"]["policy_sha256"], sha256(POLICY))
        self.assertEqual(payload["hashes"]["load_declaration_sha256"], sha256(LOAD))
        self.assertEqual(payload["hashes"]["timeout_evidence_sha256"], sha256(self.evidence))
        self.assertEqual(payload["j5"], {
            "can_id": 5, "master_id": 0, "hardware_sign": 1,
            "zero_offset_rad": 2.5389105058365757,
            "limits_rad": [-math.pi / 2, math.pi / 2],
            "control_mode": 1,
            "profile": {"pmax_rad": 12.5, "vmax_rad_s": 10.0, "tmax_nm": 28.0},
        })
        self.assertEqual(payload["timeout"]["timeout_before"], 0)
        self.assertEqual(payload["timeout"]["timeout_after"], 2000)
        self.assertEqual(payload["timeout"]["readback"], 2000)
        self.assertIs(payload["timeout"]["all_motor_state_zero"], True)
        self.assertEqual(payload["device"], DEVICE)
        self.assertEqual(payload["boot_id"], "boot-1234")
        self.assertEqual(payload["generated_at"], self.now)
        self.assertTrue((self.tmp / "artifact.json").is_file())

    def test_build_refuses_while_source_timeout_is_zero(self):
        # Use a temporary copy with J5 TIMEOUT0; the real mapping now has the
        # executed readback 2000 and must be handled by the evidence path.
        zero_mapping = write_mapping(self.tmp, timeout=0)
        self.assert_code("TIMEOUT_SOURCE_ZERO", self.build, mapping=zero_mapping)

    def test_build_rejects_zero_or_missing_timeout_evidence(self):
        for change in ({"timeout_after": 0}, {"timeout_after": None},
                       {"readback": {"rid": 9, "expected": 2000, "observed": 0,
                                     "attempts": 1, "matched": False}}):
            with self.subTest(change=change):
                document = evidence_document(self.now)
                document.update(change)
                evidence = write_evidence(self.tmp, document, "bad-timeout.json")
                self.assert_code(
                    "TIMEOUT_AFTER_INVALID" if "timeout_after" in change
                    else "TIMEOUT_EVIDENCE_READBACK",
                    self.build, evidence=evidence)

    def test_build_rejects_write_not_complete(self):
        document = evidence_document(self.now)
        document["write_outcome"] = "eagain"
        evidence = write_evidence(self.tmp, document, "eagain.json")
        self.assert_code("TIMEOUT_EVIDENCE_WRITE_OUTCOME", self.build, evidence=evidence)

    def test_build_requires_exactly_one_parameter_write(self):
        for count in (0, 2, None):
            with self.subTest(parameter_writes=count):
                document = evidence_document(self.now)
                document["parameter_writes"] = count
                evidence = write_evidence(self.tmp, document, "writes.json")
                self.assert_code("TIMEOUT_EVIDENCE_PARAMETER_WRITES",
                                 self.build, evidence=evidence)

    def test_build_rejects_nonzero_motor_state(self):
        document = evidence_document(self.now)
        document["axes"][2]["motor_state"] = 1
        evidence = write_evidence(self.tmp, document, "state.json")
        self.assert_code("TIMEOUT_EVIDENCE_MOTOR_STATE", self.build, evidence=evidence)

        document = evidence_document(self.now)
        document["all_motor_state_zero"] = False
        evidence = write_evidence(self.tmp, document, "all-zero.json")
        self.assert_code("TIMEOUT_MOTOR_STATE", self.build, evidence=evidence)

    def test_build_rejects_unsafe_side_effects(self):
        for key, code in (("enabled", "TIMEOUT_EVIDENCE_ENABLED"),
                          ("mit_sent", "TIMEOUT_EVIDENCE_MIT"),
                          ("flash_written", "TIMEOUT_FLASH_WRITTEN"),
                          ("motor_zero_written", "TIMEOUT_EVIDENCE_MOTOR_ZERO")):
            with self.subTest(key=key):
                document = evidence_document(self.now)
                document[key] = True
                evidence = write_evidence(self.tmp, document, "unsafe.json")
                self.assert_code(code, self.build, evidence=evidence)

    def test_build_rejects_stale_evidence(self):
        stale_now = self.now - 2 * admission.DEFAULT_MAX_EVIDENCE_AGE_S
        evidence = write_evidence(self.tmp, evidence_document(stale_now), "stale.json")
        self.assert_code("TIMEOUT_EVIDENCE_STALE", self.build, evidence=evidence)

    def test_build_rejects_device_mismatch(self):
        self.assert_code("EVIDENCE_DEVICE_MISMATCH", self.build, device="/dev/ttyACM9")

    def test_build_rejects_mapping_vs_evidence_mismatch(self):
        document = evidence_document(self.now)
        document["j5_id"] = 6
        self.assert_code("TIMEOUT_EVIDENCE_J5_ID", self.build,
                         evidence=write_evidence(self.tmp, document, "id.json"))

        document = evidence_document(self.now)
        document["control_mode"] = 2
        self.assert_code("TIMEOUT_EVIDENCE_MODE", self.build,
                         evidence=write_evidence(self.tmp, document, "mode.json"))

        document = evidence_document(self.now)
        document["profile"] = {"pmax_rad": 12.5, "vmax_rad_s": 45.0, "tmax_nm": 28.0}
        self.assert_code("EVIDENCE_PROFILE_MISMATCH", self.build,
                         evidence=write_evidence(self.tmp, document, "profile.json"))

    def test_build_rejects_bad_evidence_envelope(self):
        document = evidence_document(self.now)
        document["schema"] = "slaihand_j5_timeout_setup/2"
        self.assert_code("TIMEOUT_EVIDENCE_SCHEMA", self.build,
                         evidence=write_evidence(self.tmp, document, "schema.json"))

        document = evidence_document(self.now)
        document["ok"] = False
        self.assert_code("TIMEOUT_EVIDENCE_NOT_OK", self.build,
                         evidence=write_evidence(self.tmp, document, "not-ok.json"))
        self.assert_code("TIMEOUT_EVIDENCE_MISSING", self.build,
                         evidence=self.tmp / "missing-evidence.json")

    def test_build_rejects_gravity_policy(self):
        for key, value, code in (("gravity_ff_alpha", 0.1, "POLICY_GRAVITY_ENABLED"),
                                 ("gravity_float_enabled", True, "POLICY_FLOAT_ENABLED")):
            with self.subTest(key=key):
                text = POLICY.read_text(encoding="utf-8").replace(
                    "%s: %s" % (key, "0.0" if key == "gravity_ff_alpha" else "false"),
                    "%s: %s" % (key, value))
                policy = self.tmp / ("policy-%s.yaml" % key)
                policy.write_text(text, encoding="utf-8")
                self.assert_code(code, self.build, policy=policy)


# --------------------------------------------------------------- validation
class ArtifactValidateTests(AdmissionBase):
    def setUp(self):
        super().setUp()
        self.artifact = self.build()

    def test_valid_artifact_is_accepted(self):
        payload = self.validate(self.artifact)
        self.assertEqual(payload["qualification"], "j5_motion")
        self.assertEqual(payload["load_id"], "j5_3p5kg_provisional")
        # Valid without re-binding the evidence file too (embedded sha + links).
        self.validate(self.artifact, evidence=None)
        self.validate(self.artifact, max_age_s=admission.DEFAULT_MAX_AGE_S)

    def test_arbitrary_64hex_is_rejected(self):
        forged = json.loads(json.dumps(self.artifact))
        forged["sha256"] = "f" * 64
        self.assert_code("ARTIFACT_HASH_MISMATCH", self.validate, forged)

    def test_tampered_payload_is_rejected(self):
        forged = json.loads(json.dumps(self.artifact))
        forged["payload"]["j5"]["hardware_sign"] = -1
        self.assert_code("ARTIFACT_HASH_MISMATCH", self.validate, forged)

    def test_stale_artifact_is_rejected(self):
        self.assert_code(
            "ARTIFACT_STALE", self.validate, self.artifact,
            now=self.now + admission.DEFAULT_MAX_AGE_S + 1)
        self.assert_code(
            "ARTIFACT_FROM_FUTURE", self.validate, self.artifact, now=self.now - 3600)

    def test_device_mismatch_is_rejected(self):
        self.assert_code("DEVICE_MISMATCH", self.validate, self.artifact,
                         device="/dev/ttyACM9")

    def test_hash_mismatch_of_current_files_is_rejected(self):
        for label, code in (("mapping", "MAPPING_HASH_MISMATCH"),
                            ("model", "MODEL_HASH_MISMATCH"),
                            ("policy", "POLICY_HASH_MISMATCH"),
                            ("load", "LOAD_HASH_MISMATCH")):
            with self.subTest(label=label):
                sources = {"mapping": self.mapping, "model": MODEL,
                           "policy": POLICY, "load": LOAD}
                tampered = copy_file(sources[label], self.tmp,
                                     "tampered-%s" % Path(sources[label]).name)
                append_bytes(tampered)
                overrides = {label: tampered}
                self.assert_code(code, self.validate, self.artifact, **overrides)

    def test_raw_mapping_sign_zero_limits_mismatch_is_rejected(self):
        for label, code, mutate in (
                ("sign", "J5_SIGN_MISMATCH",
                 lambda payload: payload["j5"].update(hardware_sign=-1)),
                ("zero", "J5_ZERO_MISMATCH",
                 lambda payload: payload["j5"].update(zero_offset_rad=0.0)),
                ("limits", "J5_LIMITS_MISMATCH",
                 lambda payload: payload["j5"].update(limits_rad=[-0.5, 0.5]))):
            with self.subTest(label=label):
                self.assert_code(code, self.validate, reseal(self.artifact, mutate))

    def test_zero_or_missing_timeout_in_artifact_is_rejected(self):
        self.assert_code(
            "TIMEOUT_EVIDENCE_ZERO", self.validate, reseal(
                self.artifact,
                lambda payload: payload["timeout"].update(timeout_after=0)),
            evidence=None)
        self.assert_code(
            "TIMEOUT_EVIDENCE_ZERO", self.validate, reseal(
                self.artifact, lambda payload: payload["timeout"].pop("timeout_after")),
            evidence=None)
        self.assert_code(
            "TIMEOUT_MOTOR_STATE", self.validate, reseal(
                self.artifact,
                lambda payload: payload["timeout"].update(all_motor_state_zero=False)),
            evidence=None)

    def test_timeout_evidence_sha_mismatch_is_rejected(self):
        forged = reseal(
            self.artifact,
            lambda payload: payload["hashes"].update(timeout_evidence_sha256="0" * 64))
        self.assert_code("TIMEOUT_EVIDENCE_HASH_MISMATCH", self.validate, forged)

        swapped = write_evidence(self.tmp, evidence_document(self.now + 30),
                                 "other-evidence.json")
        self.assert_code("TIMEOUT_EVIDENCE_HASH_MISMATCH", self.validate,
                         self.artifact, evidence=swapped)

    def test_load_id_mismatch_is_rejected(self):
        forged = reseal(self.artifact,
                        lambda payload: payload.update(load_id="j5_3p5kg_other"))
        self.assert_code("LOAD_ID_MISMATCH", self.validate, forged)

    def test_provisional_artifact_cannot_enable_gravity_or_ff(self):
        for label, mutate, code in (
                ("formal_gravity", lambda payload: payload["load"].update(
                    formal_gravity_allowed=True), "LOAD_GRAVITY_ENABLED"),
                ("gravity_fit", lambda payload: payload["load"].update(
                    gravity_fit_allowed=True), "LOAD_GRAVITY_FIT_ENABLED"),
                ("float", lambda payload: payload["load"].update(
                    float_enabled=True), "LOAD_FLOAT_ENABLED"),
                ("alpha", lambda payload: payload["load"].update(
                    gravity_ff_alpha=0.1), "LOAD_FF_ALPHA_NONZERO"),
                ("policy_alpha", lambda payload: payload["policy"].update(
                    gravity_ff_alpha=0.1), "POLICY_GRAVITY_ENABLED"),
                ("policy_float", lambda payload: payload["policy"].update(
                    gravity_float_enabled=True), "POLICY_FLOAT_ENABLED")):
            with self.subTest(label=label):
                self.assert_code(code, self.validate, reseal(self.artifact, mutate))

    def test_load_declaration_change_is_rejected(self):
        document = yaml.safe_load(LOAD.read_text(encoding="utf-8"))
        document["formal_gravity_allowed"] = True
        load = self.tmp / "load-modified.yaml"
        load.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        # The declaration itself is rejected before it can be admitted ...
        with self.assertRaises(load_declaration.LoadDeclarationError) as context:
            load_declaration.load_declaration(load)
        self.assertEqual(context.exception.code, "LOAD_FORMAL_GRAVITY_ALLOWED")
        # ... and an artifact can never bind a modified declaration.
        self.assert_code("LOAD_HASH_MISMATCH", self.validate, self.artifact, load=load)

    def test_envelope_schema_is_enforced(self):
        wrong = json.loads(json.dumps(self.artifact))
        wrong["schema"] = "slaihand_j5_motion_calibration/2"
        path = self.tmp / "wrong-schema.json"
        path.write_text(json.dumps(wrong), encoding="utf-8")
        self.assert_code("ARTIFACT_SCHEMA", self.validate, path)


# ------------------------------------------------- candidate launch admission
class CandidateIdentityTests(AdmissionBase):
    def resolve(self, **overrides):
        arguments = dict(mapping=self.mapping, model=MODEL, policy=POLICY,
                         load=LOAD, port=DEVICE, now=self.now + 60)
        arguments.update(overrides)
        return admission.resolve_candidate_identity(
            mapping=arguments["mapping"], model=arguments["model"],
            policy=arguments["policy"], load_profile=arguments["load"],
            port=arguments["port"],
            mass_profile_sha256=arguments.get("mass_profile_sha256", ""),
            test_only_mass_profile=arguments.get("test_only_mass_profile", False),
            j5_artifact=arguments.get("j5_artifact", ""),
            timeout_evidence=arguments.get("timeout_evidence", ""),
            max_age_s=arguments.get("max_age_s", admission.DEFAULT_MAX_AGE_S),
            now=arguments["now"])

    def test_without_artifact_starts_unqualified_feedback_only(self):
        report = self.resolve()
        self.assertEqual(report["load_id"], "j5_3p5kg_provisional")
        self.assertEqual(report["mass_profile_sha256"], sha256(LOAD))
        self.assertEqual(report["j5"]["hardware_sign"], 1)
        self.assertEqual(report["j5"]["zero_offset_rad"], 2.5389105058365757)
        self.assertEqual(report["j5"]["limits_rad"], [-math.pi / 2, math.pi / 2])
        self.assertEqual(report["launch_arguments"]["j5_qualified"], "false")
        self.assertEqual(report["launch_arguments"]["j5_calibration_sha256"], "")
        self.assertIs(report["load"]["formal_gravity_allowed"], False)
        self.assertIs(report["load"]["float_enabled"], False)
        self.assertIs(report["admission"]["j5_qualified"], False)

    def test_valid_artifact_enables_qualified_j5_with_payload_sha(self):
        artifact = self.build()
        path = self.tmp / "artifact.json"
        report = self.resolve(j5_artifact=str(path),
                              timeout_evidence=str(self.evidence))
        expected = admission.payload_sha256(artifact["payload"])
        self.assertEqual(report["admission"]["j5_qualified"], True)
        self.assertEqual(report["launch_arguments"]["j5_qualified"], "true")
        self.assertEqual(report["launch_arguments"]["j5_calibration_sha256"], expected)
        self.assertEqual(report["launch_arguments"]["j5_hardware_sign"], "1")
        self.assertEqual(report["launch_arguments"]["load_id"], "j5_3p5kg_provisional")

    def test_raw_or_forged_j5_hash_is_rejected(self):
        artifact = self.build()
        forged = json.loads(json.dumps(artifact))
        forged["sha256"] = "a" * 64
        path = self.tmp / "forged.json"
        path.write_text(json.dumps(forged), encoding="utf-8")
        self.assert_code("ARTIFACT_HASH_MISMATCH", self.resolve,
                         j5_artifact=str(path))

    def test_artifact_device_mismatch_is_rejected(self):
        self.build()
        self.assert_code("DEVICE_MISMATCH", self.resolve,
                         j5_artifact=str(self.tmp / "artifact.json"),
                         port="/dev/ttyACM9")

    def test_explicit_mass_profile_hash_must_match_actual_file(self):
        good = self.resolve(mass_profile_sha256=sha256(LOAD))
        self.assertEqual(good["mass_profile_sha256"], sha256(LOAD))
        self.assert_code("MASS_PROFILE_HASH_MISMATCH", self.resolve,
                         mass_profile_sha256="c" * 64)
        test_only = self.resolve(mass_profile_sha256="c" * 64,
                                 test_only_mass_profile=True)
        self.assertEqual(test_only["mass_profile_sha256"], "c" * 64)
        self.assertIs(test_only["test_only_mass_profile"], True)

    def test_provisional_load_or_policy_cannot_enable_gravity(self):
        document = yaml.safe_load(LOAD.read_text(encoding="utf-8"))
        document["formal_gravity_allowed"] = True
        load = self.tmp / "load-gravity.yaml"
        load.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        with self.assertRaises(load_declaration.LoadDeclarationError) as context:
            self.resolve(mapping=self.mapping, load=load)
        self.assertEqual(context.exception.code, "LOAD_FORMAL_GRAVITY_ALLOWED")

        policy = self.tmp / "policy-alpha.yaml"
        policy.write_text(POLICY.read_text(encoding="utf-8").replace(
            "gravity_ff_alpha: 0.0", "gravity_ff_alpha: 0.1"), encoding="utf-8")
        self.assert_code("POLICY_GRAVITY_ENABLED", self.resolve, policy=policy)

    def test_stale_artifact_is_rejected_by_launch_admission(self):
        self.build()
        self.assert_code("ARTIFACT_STALE", self.resolve,
                         j5_artifact=str(self.tmp / "artifact.json"),
                         now=self.now + admission.DEFAULT_MAX_AGE_S + 120)


if __name__ == "__main__":
    unittest.main(verbosity=2)
