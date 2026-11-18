package org.firstinspires.ftc.teamcode.logic;

import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.PURPLE;

/**
 * The OBELISK MOTIF. Tag IDs and orders come straight from the FTC SDK's DECODE tag library
 * (Obelisk_GPP = 21, Obelisk_PGP = 22, Obelisk_PPG = 23). The motif repeats along the 9 RAMP indices.
 */
public enum Motif {
    GPP(21, GREEN, PURPLE, PURPLE),
    PGP(22, PURPLE, GREEN, PURPLE),
    PPG(23, PURPLE, PURPLE, GREEN);

    public final int tagId;
    private final ArtifactColor[] order;

    Motif(int tagId, ArtifactColor... order) {
        this.tagId = tagId;
        this.order = order;
    }

    /** Colour wanted at the given 0-based RAMP index (0..8, repeats every 3). */
    public ArtifactColor colorAt(int rampIndex) {
        return order[((rampIndex % 3) + 3) % 3];
    }

    /** @return the motif for an obelisk tag, or null if the tag is not an obelisk tag. */
    public static Motif fromTagId(int id) {
        for (Motif m : values()) if (m.tagId == id) return m;
        return null;
    }
}
