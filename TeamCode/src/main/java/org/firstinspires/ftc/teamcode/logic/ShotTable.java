package org.firstinspires.ftc.teamcode.logic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Distance -> (flywheel speed, hood position) lookup with linear interpolation between tuned points
 * and clamping outside the tuned range. Fill it with points recorded using the "Tune: Shooter" OpMode.
 */
public final class ShotTable {
    public static final class Shot {
        public final double distance;
        public final double flywheelTps;
        public final double hood;

        public Shot(double distance, double flywheelTps, double hood) {
            this.distance = distance;
            this.flywheelTps = flywheelTps;
            this.hood = hood;
        }
    }

    private final List<Shot> shots = new ArrayList<>();

    public ShotTable add(double distance, double flywheelTps, double hood) {
        if (!MathUtil.isFinite(distance) || !MathUtil.isFinite(flywheelTps) || !MathUtil.isFinite(hood)) {
            throw new IllegalArgumentException("ShotTable entries must be finite numbers");
        }
        for (Shot s : shots) {
            if (s.distance == distance) throw new IllegalArgumentException("Duplicate distance " + distance);
        }
        shots.add(new Shot(distance, flywheelTps, hood));
        Collections.sort(shots, new Comparator<Shot>() {
            @Override
            public int compare(Shot a, Shot b) {
                return Double.compare(a.distance, b.distance);
            }
        });
        return this;
    }

    public int size() {
        return shots.size();
    }

    public List<Shot> entries() {
        return Collections.unmodifiableList(shots);
    }

    public Shot lookup(double distance) {
        if (shots.isEmpty()) throw new IllegalStateException("ShotTable is empty");
        if (!MathUtil.isFinite(distance) || distance <= shots.get(0).distance) return shots.get(0);
        Shot last = shots.get(shots.size() - 1);
        if (distance >= last.distance) return last;
        for (int i = 1; i < shots.size(); i++) {
            Shot hi = shots.get(i);
            if (distance <= hi.distance) {
                Shot lo = shots.get(i - 1);
                double t = (distance - lo.distance) / (hi.distance - lo.distance);
                return new Shot(
                        distance,
                        MathUtil.lerp(lo.flywheelTps, hi.flywheelTps, t),
                        MathUtil.lerp(lo.hood, hi.hood, t));
            }
        }
        return last;
    }
}
