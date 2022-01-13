package frc.robot.utils;

import java.util.*;

/**
 * Object that holds DashboardConfig values.
 * This stores a list of DashboardKeys that are enabled,
 * and any DashboardKeys missing are considered disabled.
 * To add more DashboardKeys, simply add another Enum constant in DashboardKey.java.
 *
 * Provided methods return this object for chaining functionality.
 *
 * By default, new instances of this object will have all DashboardKeys enabled.
 * Use #onlyEnabled to enable only specifically inputted DashboardKeys.
 *
 * @see DashboardKey
 */
public class DashboardConfig
{
    private EnumSet<DashboardKey> enabledDashboardKeys;

    /**
     * By default, all DashboardKeys are enabled
     */
    public DashboardConfig()
    {
        this.enabledDashboardKeys = EnumSet.allOf(DashboardKey.class);
    }

    /**
     * Returns true if the DashboardKey is enabled, and false otherwise. Used in the BitBucketsSubsystem#updateDashboard method to determine whether to actually push the Dashboard update.
     * @param key DashboardKey to check.
     * @return true if the DashboardKey is enabled, and false if not.
     */
    public boolean isEnabled(DashboardKey key)
    {
        return this.enabledDashboardKeys.contains(key);
    }

    /**
     * Enable certain DashboardKeys. Since they are stored as a Set, there won't be duplicates.
     * @param enabled DashboardKeys to enable.
     * @return DashboardConfig object, for chaining.
     */
    public DashboardConfig addEnabled(DashboardKey... enabled)
    {
        this.enabledDashboardKeys.addAll(List.of(enabled));
        return this;
    }

    /**
     * Enable only the provided DashboardKeys. Note: THIS WILL DISABLE ALL OTHER DASHBOARDKEYS!
     * @param enabled DashboardKeys to enable.
     * @return DashboardConfig object, for chaining.
     */
    public DashboardConfig onlyEnabled(DashboardKey... enabled)
    {
        this.enabledDashboardKeys = EnumSet.copyOf(List.of(enabled));
        return this;
    }

    /**
     * Disable certain DashboardKeys safely.
     * @param disabled DashboardKeys to disable.
     * @return DashboardConfig object, for chaining.
     */
    public DashboardConfig addDisabled(DashboardKey... disabled)
    {
        this.enabledDashboardKeys.removeIf(d -> List.of(disabled).contains(d));
        return this;
    }
}
