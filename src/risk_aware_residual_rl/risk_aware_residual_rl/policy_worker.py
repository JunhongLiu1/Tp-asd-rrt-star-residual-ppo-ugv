"""Isolated SB3 JSON-lines inference worker; never imported by ROS adapter."""

import argparse
import json
import math
import sys


def _load_model(checkpoint):
    from stable_baselines3 import PPO
    return PPO.load(checkpoint, device="cpu")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    args = parser.parse_args(argv)
    model = _load_model(args.checkpoint)
    print(json.dumps({"protocol": 1, "type": "ready"}), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        request_id = request["id"]
        observation = request["observation"]
        if (
            not isinstance(request_id, int) or
            not isinstance(observation, list) or
            len(observation) != 14 or
            not all(math.isfinite(float(value)) for value in observation)
        ):
            raise ValueError("invalid inference request")
        action, unused_state = model.predict(
            observation,
            deterministic=bool(request.get("deterministic", True)),
        )
        del unused_state
        if hasattr(action, "tolist"):
            action = action.tolist()
        if len(action) == 1 and hasattr(action[0], "__len__"):
            action = action[0]
        print(json.dumps({
            "action": [float(action[0]), float(action[1])],
            "id": request_id,
        }, separators=(",", ":")), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
