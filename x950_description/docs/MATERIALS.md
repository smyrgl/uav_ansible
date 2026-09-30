# Visual materials

The material pass replaces the early orange/teal/blue/green identification colors with restrained engineering finishes. The catalog is `config/materials.json`; apply it inside Blender with `scripts/apply_materials.py` after adding the generated visual meshes and before exporting/rendering.

The user confirmed **matte black PAHT-CF** for printed sensor and GNSS brackets, **black TPU95** for the battery cradle/end block, **silver plastic** for the battery wrap, **carbon fiber** for the main frame tubes, and **mixed-color composite** for the frame panels. The user-supplied manufacturer photograph guides the silver-gray top cover and darker graphite side panels. Silver battery wrap and composite cover are modeled as dielectrics with zero metallic response. Carbon tubes have a dark resin finish without a fabricated weave pattern.

The user also confirmed the **Avia silver metal housing**, **D555 black metal housing**, and **black plastic GNSS radomes**. Exact alloys, coatings, polymers and sheen remain unknown. These corrections keep the existing catalog keys for compatibility; the readable metadata records the confirmed material families and colors.

Exact panel fiber/resin systems, black-anodized fitting appearance, the E1R housing material and all numerical shader roughness values remain unconfirmed. Colors taken from the reference photograph are appearance approximations, not colorimetric measurements. The original source meshes can combine several physical components, so assigning a finish does not establish a homogeneous engineering material for the entire mesh. Carbon tube assignments similarly do not identify any integral metal fittings.

| Geometry | Appearance and evidence |
| --- | --- |
| Avia/D555/E1R printed mounts, GNSS brackets | Confirmed PAHT-CF, matte black |
| Battery cradle and end block | Confirmed TPU95, black |
| Battery wrap | Confirmed silver plastic; exact polymer unknown |
| Main frame arms, struts and landing tubes | Confirmed carbon-fiber tube material; sheen/layup unmeasured |
| Main plates | Confirmed composite construction; graphite finish approximation |
| Motor/fitting/tab meshes | Provisional black metallic finish; exact alloy/coating unknown |
| Main body / top cover | Confirmed composite construction; photo-guided graphite / silver-gray |
| Avia one-piece CAD mesh | Confirmed silver metal housing; exact alloy/finish and optical regions unresolved |
| D555 one-piece CAD mesh | Confirmed black metal housing; exact alloy/coating and optical regions unresolved |
| E1R one-piece CAD mesh | Neutral housing finish; exact material and optical regions unresolved |
| GNSS radomes | Confirmed black plastic; exact polymer/finish unknown |
| Radio antennas | Provisional black polymer finish |
| Holybro combined assembly STL | Provisional neutral mixed-material overview; no component-level PCB/metal colors inferred |
| Hadron housing and lens/barrel proxies | Provisional dark housing and illustrative dark optics; no glass, germanium or optical-coating claim |

Every assigned Object and Material carries readable `engineering_material`, `material_status`, and `appearance_status` custom properties. Unmatched visual meshes receive a clearly labeled neutral fallback and are returned in `result['unclassified_visuals']`. Add a rule to the catalog, or set a generated object's `material_key_override` to an existing catalog key, for a deliberate exception.

Each finish uses a simple Principled shader and the same base RGBA in `material.diffuse_color`, so LinkForge's URDF export and RViz retain the intended opaque color. RViz's STL material does not reproduce Blender roughness or metallic response. Workbench review renders also show a simplified material appearance; full shader inspection requires Blender Material Preview or a shader-capable render engine.

The script is idempotent and accepts only the generated `X950_ROS_LinkForge` scene. It leaves the original source scene, geometry, poses, coverage guides, and physical properties unchanged. Shared mesh data is copied before replacing material slots; shared Objects or managed finishes outside the generated visuals cause a clear refusal instead of modifying the source. It does not assign density, calculate inertia, change mass metadata, save the Blender file, or export on its own.

Reapply the script after adding geometry or changing the catalog, then run the normal export/render/save pipeline. The finished catalog is an appearance record, not a manufacturing bill of materials or a physics model.
