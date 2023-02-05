// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class FFDisplay extends SubsystemBase {

    public static NetworkTable table;

    public FFDisplay(String tableName) {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Gt the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table calledtable.
        table = inst.getTable(tableName);

    }

    

    @Override
    public void periodic() {

    }

    /**
     * Gets the entry for the specified key.
     *
     * @param key the key name
     * @return Network table entry.
     */
    public NetworkTableEntry getEntry(String key) {
        return table.getEntry(key);
    }

    /**
     * Checks the table and tells if it contains the specified key.
     *
     * @param key the key to search for
     * @return true if the table as a value assigned to the given key
     */
    public boolean containsKey(String key) {
        return table.containsKey(key);
    }

    /**
     * Put a boolean in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public boolean putBoolean(String key, boolean value) {
        return getEntry(key).setBoolean(value);
    }

    /**
     * Put a number in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public boolean putNumber(String key, double value) {
        return getEntry(key).setDouble(value);
    }

    /**
     * Put a string in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public boolean putString(String key, String value) {
        return getEntry(key).setString(value);
    }

}
