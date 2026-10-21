"""Hardware-independent H2R hand identity and deliberate gesture confirmation."""


def unique_hand(results, label, minimum_score=0.7):
    """Reject duplicate or low-confidence identities instead of selecting by position."""
    matches = []
    for landmarks, handedness in zip(results.multi_hand_landmarks or (),
                                      results.multi_handedness or ()):
        if handedness.classification:
            identity = handedness.classification[0]
            if identity.label == label:
                matches.append((landmarks, identity.score))
    if len(matches) != 1 or matches[0][1] < minimum_score:
        return None, None
    return matches[0]


def hands_separated(left, right):
    """Conservatively reject overlapping image-space landmark bounds."""
    if left is None or right is None:
        return True
    bounds = []
    for hand in (left, right):
        xs = [lm.x for lm in hand.landmark]
        ys = [lm.y for lm in hand.landmark]
        bounds.append((min(xs), min(ys), max(xs), max(ys)))
    a, b = bounds
    return min(a[2], b[2]) <= max(a[0], b[0]) or min(a[3], b[3]) <= max(a[1], b[1])


class FistConfirmation:
    """Require an open hand then a fresh two-second fist for each workflow stage."""

    def __init__(self):
        self.context = None
        self.previous_time = None
        self.open_since = None
        self.fist_since = None
        self.armed = False

    def update(self, context, *, visible, fist, open_palm, now, ready=True):
        gap = self.previous_time is not None and now - self.previous_time > 0.5
        if context != self.context or gap or not visible or context is None:
            self.context = context
            self.open_since = self.fist_since = None
            self.armed = False
        self.previous_time = now
        if context is None or not visible:
            return False
        if not self.armed:
            if open_palm:
                if self.open_since is None:
                    self.open_since = now
                self.armed = now - self.open_since >= 0.3
            else:
                self.open_since = None
            return False
        if not ready:
            self.fist_since = None
            return False
        if not fist:
            self.fist_since = None
            return False
        if self.fist_since is None:
            self.fist_since = now
        if now - self.fist_since < 2.0:
            return False
        self.armed = False
        self.open_since = self.fist_since = None
        return True

    def feedback(self, now):
        """Return the same timer used for confirmation, not a separate UI timer."""
        if self.fist_since is not None:
            seconds = max(0.0, min(2.0, now - self.fist_since))
            return f"Hold RIGHT FIST: {seconds:.1f}/2.0s", seconds / 2.0
        if self.armed:
            return "Hold RIGHT FIST for 2 s", 0.0
        return "Open RIGHT palm to arm, then hold fist for 2 s", 0.0


def segmentation_rejection_reason(masks):
    """A hand prompt is insufficient: both tracked masks must remain valid."""
    hand, obj = masks.get(1), masks.get(2)
    if hand is None or not hand.any():
        return "Hand mask missing: reselect hand/object"
    if obj is None or not obj.any():
        return "Object mask missing: reselect object"
    if hand.shape != obj.shape:
        return "Hand/object masks do not match"
    return None


def manual_segmentation_prompts(hand, obj):
    """Keep the positive hand seed outside the selected object; cross-label negatives.

    A palm-centre seed may sit on a held object. A box enclosing the entire
    hand cannot distinguish the two and must be tightened or replaced by a click.
    Returns SAM2 keyword arguments without importing the model or GPU runtime.
    """
    def outside_object(point):
        x, y = point
        if "bbox" in obj:
            x1, y1, x2, y2 = obj["bbox"]
            return not (x1 - 5 <= x <= x2 + 5 and y1 - 5 <= y <= y2 + 5)
        ox, oy = obj["point"]
        return (x - ox) ** 2 + (y - oy) ** 2 > 12 ** 2

    candidates = hand.get("point_candidates", [hand["point"]])
    seed = next((point for point in candidates if outside_object(point)), None)
    if seed is None:
        raise ValueError("Object selection covers hand seeds: tighten box or click OBJECT")
    hand["point"] = seed
    hand_args = {"points": [seed], "labels": [1]}
    object_args = {"points": [seed], "labels": [0]}
    if "point" in obj:
        hand_args["points"].append(obj["point"])
        hand_args["labels"].append(0)
        object_args["points"].insert(0, obj["point"])
        object_args["labels"].insert(0, 1)
    else:
        x1, y1, x2, y2 = obj["bbox"]
        object_args["bbox"] = [[x1, y1], [x2, y2]]
    return hand_args, object_args
