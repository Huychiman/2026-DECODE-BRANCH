package org.firstinspires.ftc.teamcode.logic;

import java.util.ArrayList;
import java.util.List;

/**
 * Chooses the order to fire the sorter's 3 slots so CLASSIFIED artifacts match the MOTIF (2 bonus
 * PATTERN points each). Without a known motif or known colours it simply fires every loaded slot,
 * starting from the current one and minimising sorter travel.
 */
public final class FirePlanner {
    private FirePlanner() {}

    /**
     * @param slots colour in each sorter slot (index 0..2), EMPTY slots are skipped
     * @param motif obelisk motif, or null if unknown
     * @param rampCount artifacts already on the RAMP (sets which motif index comes next)
     * @param currentSlot slot currently lined up with the kicker (0..2)
     * @return slot indices in firing order
     */
    public static List<Integer> plan(ArtifactColor[] slots, Motif motif, int rampCount, int currentSlot) {
        boolean[] used = new boolean[slots.length];
        List<Integer> order = new ArrayList<>();
        int at = currentSlot;
        int loaded = 0;
        for (ArtifactColor c : slots) if (c != ArtifactColor.EMPTY) loaded++;

        for (int n = 0; n < loaded; n++) {
            ArtifactColor wanted = motif == null ? null : motif.colorAt(rampCount + n);
            int best = -1;
            int bestScore = Integer.MAX_VALUE;
            for (int i = 0; i < slots.length; i++) {
                if (used[i] || slots[i] == ArtifactColor.EMPTY) continue;
                // Score: wrong known colour is worst, unknown colour is a gamble, right colour best.
                int colorPenalty;
                if (wanted == null || slots[i] == wanted) colorPenalty = 0;
                else if (slots[i] == ArtifactColor.UNKNOWN) colorPenalty = 10;
                else colorPenalty = 20;
                int score = colorPenalty + Math.abs(i - at);
                if (score < bestScore) {
                    bestScore = score;
                    best = i;
                }
            }
            used[best] = true;
            order.add(best);
            at = best;
        }
        return order;
    }
}
