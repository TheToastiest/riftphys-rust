// riftphys-materials/src/vis.rs
use crate::MaterialId;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MatVis {
    pub base_rgb: [f32; 3],  // linear (not sRGB)
    pub rough: f32,          // 0..1
    pub metal: f32,          // 0..1
}

#[inline]
fn srgb_to_lin(c: f32) -> f32 {
    if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}
#[inline]
fn srgb8(r: u8, g: u8, b: u8) -> [f32; 3] {
    [srgb_to_lin(r as f32 / 255.0),
        srgb_to_lin(g as f32 / 255.0),
        srgb_to_lin(b as f32 / 255.0)]
}

/// Muted defaults per material. Metals use `metal≈1` (PBR).
pub fn mat_vis(id: MaterialId) -> MatVis {
    use MaterialId::*;
    match id {
        // --- Core ---
        Default  => mv(120,130,128, 0.85, 0.00),
        Ice      => mv(196,214,230, 0.25, 0.00),
        Rubber   => mv( 34, 34, 34, 0.95, 0.00),
        Steel    => mv(150,155,162, 0.40, 1.00),
        Grit     => mv( 95, 98,102, 0.95, 0.00),
        Wood     => mv(132,106, 84, 0.80, 0.00),
        Concrete => mv(126,127,129, 0.90, 0.00),
        Asphalt  => mv( 58, 60, 62, 0.92, 0.00),
        Leather  => mv(120, 84, 55, 0.60, 0.00),
        Skin     => mv(220,190,175, 0.60, 0.00),
        Carpet   => mv(110,110,110, 0.95, 0.00),
        Glass    => mv(200,205,210, 0.08, 0.00),
        Titanium => mv(170,175,180, 0.35, 1.00),
        Aluminum => mv(180,185,192, 0.45, 1.00),
        Marble   => mv(222,224,228, 0.70, 0.00),
        Diamond  => mv(230,245,255, 0.02, 0.00),

        // --- Wet variants ---
        ConcreteWet => mv(105,108,112, 0.55, 0.00),
        AsphaltWet  => mv( 45, 48, 52, 0.50, 0.00),

        // --- Metals ---
        StainlessSteel   => mv(172,176,182, 0.35, 1.00),
        CastIron         => mv( 92, 94, 96, 0.50, 1.00),
        WroughtIron      => mv(110,114,118, 0.45, 1.00),
        CarbonSteel      => mv(140,146,152, 0.42, 1.00),
        Copper           => mv(184,120, 90, 0.35, 1.00),
        Brass            => mv(190,170,110, 0.35, 1.00),
        Bronze           => mv(150,118, 80, 0.40, 1.00),
        TitaniumAlloy    => mv(168,172,178, 0.38, 1.00),
        Magnesium        => mv(190,192,195, 0.55, 1.00),
        Nickel           => mv(160,165,170, 0.40, 1.00),
        Chromium         => mv(210,215,220, 0.20, 1.00),
        Zinc             => mv(180,190,200, 0.55, 1.00),
        Tin              => mv(170,175,180, 0.55, 1.00),
        Lead             => mv( 85, 88, 92, 0.60, 1.00),
        Gold             => mv(212,175, 55, 0.28, 1.00),
        Silver           => mv(210,214,218, 0.25, 1.00),
        Platinum         => mv(196,200,206, 0.25, 1.00),
        AnodizedAluminum => mv(120,160,190, 0.40, 1.00),

        // --- Stones / Minerals ---
        Granite    => mv(128,132,135, 0.90, 0.00),
        Limestone  => mv(186,182,170, 0.88, 0.00),
        Sandstone  => mv(184,168,146, 0.88, 0.00),
        Slate      => mv( 83, 90,100, 0.92, 0.00),
        Basalt     => mv( 60, 62, 66, 0.94, 0.00),
        Obsidian   => mv( 30, 32, 36, 0.10, 0.00), // glassy
        Travertine => mv(200,190,176, 0.88, 0.00),
        Quartz     => mv(230,232,238, 0.12, 0.00),

        // --- Gemstones (polished, low rough) ---
        Ruby       => mv(145, 20, 35, 0.10, 0.00),
        Sapphire   => mv( 30, 55,120, 0.10, 0.00),
        Emerald    => mv( 30,110, 80, 0.12, 0.00),
        Topaz      => mv(210,165, 95, 0.12, 0.00),
        Amethyst   => mv(110, 70,145, 0.12, 0.00),
        Citrine    => mv(210,160, 70, 0.12, 0.00),
        Aquamarine => mv(125,175,185, 0.12, 0.00),
        Garnet     => mv(120, 35, 40, 0.12, 0.00),
        Opal       => mv(215,220,225, 0.18, 0.00),
        Jade       => mv( 90,135,100, 0.16, 0.00),
        Onyx       => mv( 20, 22, 24, 0.12, 0.00),
        Turquoise  => mv( 50,160,165, 0.16, 0.00),
        Peridot    => mv(140,175, 80, 0.16, 0.00),
        Moonstone  => mv(210,215,230, 0.16, 0.00),

        // --- Ceramics & Glass ---
        TemperedGlass     => mv(205,210,215, 0.06, 0.00),
        BorosilicateGlass => mv(200,206,212, 0.07, 0.00),
        Porcelain         => mv(232,234,236, 0.18, 0.00),
        CeramicGlazed     => mv(230,232,234, 0.20, 0.00),
        CeramicUnglazed   => mv(210,200,190, 0.85, 0.00),
        Earthenware       => mv(170,135,110, 0.90, 0.00),
        Stoneware         => mv(185,170,155, 0.88, 0.00),
        TileGlazed        => mv(225,230,235, 0.18, 0.00),
        TileUnglazed      => mv(200,190,180, 0.88, 0.00),

        // --- Polymers ---
        PTFE          => mv(238,240,242, 0.20, 0.00),
        UHMWPE        => mv(230,236,236, 0.30, 0.00),
        HDPE          => mv(205,210,220, 0.35, 0.00),
        LDPE          => mv(205,210,220, 0.45, 0.00),
        Polypropylene => mv(210,215,220, 0.45, 0.00),
        POM_Delrin    => mv( 30, 30, 30, 0.55, 0.00),
        Nylon         => mv(220,225,230, 0.60, 0.00),
        ABS           => mv(210,210,210, 0.65, 0.00),
        PVC_Rigid     => mv(200,205,210, 0.55, 0.00),
        Polycarbonate => mv(210,215,220, 0.28, 0.00),
        Acrylic_PMMA  => mv(220,225,230, 0.22, 0.00),
        PEEK          => mv(190,175,160, 0.70, 0.00),
        PET           => mv(210,215,220, 0.50, 0.00),
        Polyurethane  => mv(220,220,210, 0.70, 0.00),

        // --- Rubbers ---
        RubberSoft     => mv( 28, 28, 28, 0.96, 0.00),
        RubberHard     => mv( 40, 40, 40, 0.94, 0.00),
        SiliconeRubber => mv(220,220,220, 0.90, 0.00),
        NeopreneRubber => mv( 32, 36, 40, 0.95, 0.00),
        NitrileRubber  => mv( 36, 36, 36, 0.95, 0.00),

        // --- Woods ---
        Oak     => mv(164,130, 92, 0.80, 0.00),
        Maple   => mv(195,165,125, 0.80, 0.00),
        Pine    => mv(210,190,140, 0.82, 0.00),
        Birch   => mv(220,200,170, 0.80, 0.00),
        Bamboo  => mv(195,175,120, 0.82, 0.00),
        Teak    => mv(140,100, 70, 0.78, 0.00),
        Plywood => mv(190,165,130, 0.82, 0.00),
        MDF     => mv(170,150,120, 0.85, 0.00),

        // --- Fabrics / Soft ---
        Cotton    => mv(230,230,228, 0.92, 0.00),
        Wool      => mv(185,180,175, 0.94, 0.00),
        Denim     => mv( 70, 90,120, 0.95, 0.00),
        Silk      => mv(230,225,220, 0.35, 0.00),
        Polyester => mv(210,210,210, 0.90, 0.00),
        Felt      => mv(140,140,140, 0.96, 0.00),
        Canvas    => mv(190,185,175, 0.92, 0.00),

        // --- Granular / Soils ---
        SandDry   => mv(200,185,150, 0.88, 0.00),
        SandWet   => mv(160,145,120, 0.65, 0.00),
        ClayDry   => mv(170,140,110, 0.90, 0.00),
        ClayWet   => mv(120, 95, 75, 0.75, 0.00),
        SnowFresh => mv(238,242,246, 0.80, 0.00),
        SnowPacked=> mv(220,224,232, 0.72, 0.00),
        Gravel    => mv(120,120,120, 0.92, 0.00),
        Mud       => mv( 96, 78, 64, 0.95, 0.00),

        // --- Paper / Card ---
        Paper               => mv(235,236,232, 0.92, 0.00),
        Cardboard           => mv(200,175,140, 0.92, 0.00),
        CorrugatedCardboard => mv(190,160,120, 0.93, 0.00),

        // --- Composites / Misc ---
        CarbonFiberEpoxy => mv( 24, 24, 24, 0.35, 0.00),
        Fiberglass       => mv(200,205,210, 0.65, 0.00),
        GripTape         => mv( 18, 18, 18, 0.98, 0.00),
        PaintedSteel     => mv(150,155,162, 0.60, 0.05),  // paint → not fully metallic
        EVA_Foam         => mv(210,210,210, 0.90, 0.00),
        PU_Foam          => mv(235,235,220, 0.92, 0.00),
    }
}

#[inline]
fn mv(r: u8, g: u8, b: u8, rough: f32, metal: f32) -> MatVis {
    MatVis { base_rgb: srgb8(r,g,b), rough, metal }
}
