# Collision Examples

Collision checking below the high-level `Robot` API: contact-manager plugins,
collision margins, contact validation and continuous checks. Each example is a
port of an upstream tesseract example or test, or takes its parameters from
upstream source, cited in the module docstring. Every value printed is also
asserted in `tests/examples`.

```bash
# Installed console script
tesseract_contact_managers_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.contact_managers_example
```

## Contact Managers

A port of upstream's `box_box_example.cpp`, run on every discrete backend, then
the rest of the contact-manager API on the same scene.

Load the plugin factory from `contact_manager_plugins.yaml`. It names three
discrete managers (`BulletDiscreteBVHManager` is the default) and two
continuous ones (`BulletCastBVHManager` is the default):

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:factory"
```

The second box is upstream's `box_2m.ply` as a convex hull. Upstream reads the
PLY with `loadSimplePlyFile`, which is not bound; `createMeshFromPath` loads the
same mesh, and `makeConvexMesh` is the call upstream makes on it:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:hull"
```

The example itself: a unit box inside the 2 m hull, then moved to x = 1.6, a
0.1 m gap, and checked at margins 0.1 and 0.25.

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:box_box"
```

| backend | inside: pair | inside: distance | x = 1.6, margin 0.1 | x = 1.6, margin 0.25 |
|---|---|---|---|---|
| `BulletDiscreteBVHManager` | box_link / second_box_link | −1.2663 | no contact | 0.1000 |
| `BulletDiscreteSimpleManager` | box_link / second_box_link | −1.2663 | no contact | 0.1000 |
| `FCLDiscreteBVHManager` | box_link / second_box_link | −1.3000 | no contact | 0.1000 |

The backends agree on which pair collides, not on how deep: the analytic
penetration is 1.3 m, which FCL reports. Upstream's own box-box unit test
avoids `box_2m.ply` for Bullet for this reason (`collision_box_box_unit.hpp:58`).
`BulletDiscreteSimpleManager` also reports `link_names` in the other order, so
the example compares the `ContactResultMap` keys, which are always ordered.

!!! note "The margin 0.1 case is a boundary"
    Both backends keep a contact whose distance is *at most* the margin
    (`bullet_utils.cpp:1072`, `fcl_utils.cpp:307`). In doubles the gap is
    `(1.6 - 0.5) - 1.0 = 0.10000000000000009`, one ulp above 0.1, so margin
    0.1 reports nothing, as upstream's unit test expects
    (`collision_box_box_unit.hpp:201`). Any margin even 1e-12 m larger reports
    the pair.

A manager can also be created from a `PluginInfo`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:plugin_info"
```

### Collision margins

The pair-margin cases of upstream's box-box unit test, built with the
pair-data constructors and checked by a contact test at the 0.1 m gap pose. A
margin on a pair that isn't in the scene (1.7 m) raises
`getMaxCollisionMargin()` but not `getMaxCollisionMargin("box_link")`, so the
boxes stay out of contact. `incrementMargins(0.15)` and `scaleMargins(2.5)`
take 0.1 to upstream's second margin, 0.25. `apply` with `MODIFY` merges pair
margins and with `REPLACE` swaps them:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:margins"
```

| case | max | max for `box_link` | contact |
|---|---|---|---|
| default 0.1 + unrelated pair 1.7 | 1.7 | 0.1 | no |
| default 0.1 + box pair 0.25 | 0.25 | 0.25 | yes |
| box pair 0.25 only | 0.25 | 0.25 | yes |
| 0.1, `incrementMargins(0.15)` | 0.25 | 0.25 | yes |
| 0.1, `scaleMargins(2.5)` | 0.25 | 0.25 | yes |
| unrelated 1.7, `apply(box pair 0.25, MODIFY)` | 1.7 | 0.25 | yes |
| unrelated 1.7, `apply(box pair 0.25, REPLACE)` | 0.25 | 0.25 | yes |

`CollisionMarginPairData` has the same arithmetic. Its
`getMaxCollisionMargin(link)` returns `None` for a link no pair involves:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:pair_data"
```

### Editing the factory

Add a copy of a plugin, make it the default, create a manager from it and
remove it again, as upstream's `PluginFactorAPIUnit` does. A manager created by
plugin name carries that name. When the default is removed, the first remaining
plugin by name becomes the default:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:edit_plugins"
```

`saveConfig` writes the factory's current configuration, and a factory built
from that file reports the same `getConfig()`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:save_config"
```

`clearSearchPaths` and `clearSearchLibraries` empty the plugin loader's
search lists:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:clear_search"
```

### Allowed collision matrix

Upstream's `TestGetAllowedCollisions` entries, built from a dict keyed by
`makeOrderedLinkPair`. The result equals an ACM built entry by entry with
`addAllowedCollision`. `getAllowedCollisions` lists the links allowed to collide
with any of `link1`, `link3`, `cause_duplicate`: 5 names, or 6 when duplicates
are kept. `removeAllowedCollision("link1")` drops every entry involving `link1`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:acm"
```

### Contact validation and result maps

A `ContactResultValidator` subclass approves or rejects each contact. A contact
test calls it from its worker thread with the GIL taken again, and an exception
it raises propagates to the caller:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:validator_class"
```

With the thin box enabled, the three objects overlap pairwise. The validator is
called once directly, then set on `ContactRequest.is_valid`, where it drops the
`box_link` / `thin_box_link` pair:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:validator"
```

A `ContactResultMap` iterates as `(key, ContactResultVector)` pairs. `filter`
edits the vectors in place, and `addContactResult` / `setContactResult` take
either one result or a vector:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:result_map"
```

### Continuous (cast) manager

Upstream's box-box cast unit test: a 0.25 m box swept from (−1.9, 0, 0) to
(1.9, 3.8, 0) past a static unit box. Each of the three cast overloads of
`setCollisionObjectsTransform` sets the same sweep, by name, by name list or by
a pair of transform maps:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast"
```

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast_names"
```

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast_maps"
```

Each gives upstream's values: distance −0.2475, `cc_time` (−1, 0.25), and the
contact point mapped through the sweep's start and end poses,
`transform[1] * nearest_points_local[1]` = (−1.275, −0.625, 0) and
`cc_transform[1] * nearest_points_local[1]` = (2.525, 3.175, 0).

!!! note "Parking a cast object"
    A single-pose `setCollisionObjectsTransform` on a cast manager moves the
    object but keeps its previous sweep (`bullet_cast_bvh_manager.cpp:240–256`).
    To park an object, set a zero-length sweep: the same pose twice.

### Environment contact managers

On the IIWA environment, `EnvironmentContactAllowedValidator` answers from the
scene graph's ACM: `base_link` / `link_1` is disabled in the SRDF ("Adjacent"),
`base_link` / `link_5` is not. Any configured manager can be fetched by name,
and the `SetActive*ContactManagerCommand` pair switches the active ones:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:environment"
```
