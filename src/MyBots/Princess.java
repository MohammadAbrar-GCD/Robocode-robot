package MyBots;

import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class Princess extends AdvancedRobot {
    // Variables and constants
    boolean peek;
    double moveAmount;
    String targetName;
    double targetBearing;
    double targetDistance;
    boolean hitByBullet;
    double scanDir = 1; // Initial scan direction is positive
    Map<String, RobotData> enemyMap = new HashMap<>();
    RobotData target;
    Color defaultColor = Color.PINK; // Set default color
    Color dealingDamageColor = Color.RED; // Color when dealing damage
    int direction = 1; // Initial movement direction
    long lastDirectionShift = 0; // Timestamp of the last direction shift

    private ScannedRobotEvent oldestScanned;
    // Constants for dodging
    private static final double NEARBY_BULLET_DISTANCE_THRESHOLD = 200; // Set your desired nearby bullet distance threshold
    private static final int MAX_DODGING_ATTEMPTS = 3; // Maximum dodging attempts allowed
    private static final double MIN_ENERGY_THRESHOLD = 20; // Minimum energy threshold for dodging
    private static final long MAX_TIME_ALLOWED = 100; // Maximum time allowed for dodging in ticks
    private static final double GUN_AIMING_THRESHOLD = Math.toRadians(1); // Threshold for gun aiming

    // Other variables
    private double dodgeAngle; // Angle for dodging
    private int dodgingAttempts; // Number of dodging attempts
    private long numberOfIterations; // Number of iterations in the main loop

    // Maximum size for the enemyMap
    private static final int MAX_ENEMY_MAP_SIZE = 10;
    private static final double HALF_ROBOT_SIZE = 0;
    private static final double PREDICTION_TIME = 0;

    public void run() {
        // Set initial color
        setColors(defaultColor, defaultColor, defaultColor);

        // Robot main loop
        while (true) {
            // Update scan direction
            handleRadar();

            // Execute predictive dodging
            predictiveDodging();

            // Execute predictive aiming and firing
            handleGun();
            
            fireGunWhenReady();

            // Move along walls
            moveRobot();

            // Increment number of iterations
            numberOfIterations++;

            // Break the loop after a certain number of iterations
            int MAX_ITERATIONS = 1000;
            if (numberOfIterations >= MAX_ITERATIONS) {
                break;
            }
        }
    }

    public void setColors(Color bodyColor, Color gunColor, Color radarColor) {
        setBodyColor(bodyColor);
        setGunColor(gunColor);
        setRadarColor(radarColor);
    }

    public Bullet getNearestBullet(ArrayList<Bullet> bullets) {
        if (bullets == null || bullets.isEmpty()) {
            return null;
        }

        Bullet nearestBullet = bullets.get(0);
        double minDistance = distance(nearestBullet);

        for (int i = 1; i < bullets.size(); i++) {
            double dist = distance(bullets.get(i));
            if (dist < minDistance) {
                minDistance = dist;
                nearestBullet = bullets.get(i);
            }
        }

        return nearestBullet;
    }

    private double distance(Bullet bullet) {
        double dx = bullet.getX() - getX();
        double dy = bullet.getY() - getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void predictiveDodging() {
        try {
            boolean hasDodged = false;

            // Your dodging logic goes here
            // Placeholder for dodging logic
            Bullet closestBullet = getNearestBullet(null); // Assume you have a method to get the nearest bullet

            if (closestBullet != null) {
                double bulletX = closestBullet.getX();
                double bulletY = closestBullet.getY();
                double myX = getX();
                double myY = getY();
                double distanceToBullet = Math.sqrt(Math.pow(bulletX - myX, 2) + Math.pow(bulletY - myY, 2));

                if (distanceToBullet < NEARBY_BULLET_DISTANCE_THRESHOLD) {
                    // Calculate time to impact
                    long timeToImpact = (long) (distanceToBullet / closestBullet.getVelocity());

                    // Predict future position of the bullet
                    double futureX = bulletX + Math.sin(closestBullet.getHeadingRadians()) * closestBullet.getVelocity() * timeToImpact;
                    double futureY = bulletY + Math.cos(closestBullet.getHeadingRadians()) * closestBullet.getVelocity() * timeToImpact;

                    // Calculate dodge angle
                    double angleToBullet = Math.atan2(futureX - myX, futureY - myY);
                    double headingRadians = getHeadingRadians();
                    double relativeAngle = Utils.normalRelativeAngle(angleToBullet - headingRadians);
                    dodgeAngle = relativeAngle - Math.PI / 2; // Adjust as needed
                }
            }

            // Add an exit condition based on the flag or other criteria
            if (hasDodged || dodgingAttempts >= MAX_DODGING_ATTEMPTS || getEnergy() < MIN_ENERGY_THRESHOLD || getTime() >= MAX_TIME_ALLOWED) {
                return; // Exit the method
            }
        } catch (Exception e) {
            // Handle any unexpected exceptions
            System.err.println("An error occurred during predictive dodging: " + e.getMessage());
            e.printStackTrace(); // Print stack trace for debugging
        }
    }

    private void updateGunDirection() {
        if (target != null) {
            double absoluteBearing = getHeadingRadians() + target.targetBearing;
            setTurnGunRightRadians(Utils.normalRelativeAngle(absoluteBearing - getGunHeadingRadians()));
        }
    }

    private void handleGun() {
        if (getGunHeat() == 0 && target != null) {
            double futureX = predictFutureX(target);
            double futureY = predictFutureY(target);
            double dx = futureX - getX();
            double dy = futureY - getY();
            double absoluteBearing = Math.atan2(dx, dy);
            double gunTurn = Utils.normalRelativeAngle(absoluteBearing - getGunHeadingRadians());
            if (Math.abs(gunTurn) < GUN_AIMING_THRESHOLD) {
                System.out.println("Firing gun!");
                fire(2);
            }
        }
    }

    private void fireGunWhenReady() {
        if (getGunHeat() == 0 && target != null) {
            double futureX = predictFutureX(target);
            double futureY = predictFutureY(target);
            double dx = futureX - getX();
            double dy = futureY - getY();
            double absoluteBearing = Math.atan2(dx, dy);
            double gunTurn = Utils.normalRelativeAngle(absoluteBearing - getGunHeadingRadians());
            if (Math.abs(gunTurn) < GUN_AIMING_THRESHOLD) {
                fire(2); // Adjust firepower as needed
            }
        }
    }

    private void moveRobot() {
        try {
            double x = getX();
            double y = getY();
            double width = getBattleFieldWidth();
            double height = getBattleFieldHeight();

            // Calculate distances to walls
            double distanceToTopWall = y;
            double distanceToBottomWall = height - y;
            double distanceToLeftWall = x;
            double distanceToRightWall = width - x;

            // Determine the minimum distance to any wall
            double minDistanceToWall = Math.min(Math.min(distanceToTopWall, distanceToBottomWall),
                    Math.min(distanceToLeftWall, distanceToRightWall));

            // Calculate a turning factor based on the distance to the nearest wall
            double turningFactor = 1.0 - (minDistanceToWall / Math.min(width, height));

            // Adjust the robot's movement based on the turning factor
            setMaxVelocity(Rules.MAX_VELOCITY * turningFactor);

            // Move the robot ahead
            ahead(Double.POSITIVE_INFINITY); // Move ahead indefinitely
        } catch (Exception e) {
            // Handle any unexpected exceptions
            System.err.println("An error occurred while moving robot: " + e.getMessage());
            e.printStackTrace(); // Print stack trace for debugging
        }
    }

    public void onHitWall(HitWallEvent event) {
        // Handle the event by reversing the robot's direction
        setBack(100); // Move back by 100 units
        // Optionally, you can also adjust the robot's heading here to avoid getting stuck
        turnRight(90); // Turn right by 90 degrees after hitting the wall
    }

    public void handleRadar() {
        // If the oldest scanned robot is available and there is data for all robots in the enemy map
        if (enemyMap.size() > 0) {
            // Extract information from the oldest scanned robot event
            RobotData oldestScanned = enemyMap.entrySet().iterator().next().getValue();
            double distance = oldestScanned.targetDistance;
            double bearingRadians = oldestScanned.targetBearing;
            double ourHeading = getHeadingRadians();
            double ourX = getX();
            double ourY = getY();

            // Calculate the absolute bearing of the oldest scanned robot
            double absoluteBearing = ourHeading + bearingRadians;

            // Calculate the enemy's X and Y coordinates relative to our robot's position
            double enemyX = ourX + distance * Math.sin(absoluteBearing);
            double enemyY = ourY + distance * Math.cos(absoluteBearing);

            // Calculate the bearing to the oldest scanned robot
            double bearing = bearingTo(ourHeading, enemyX, enemyY);

            // Update the scan direction based on the bearing
            scanDir = bearing;
        }
    }


    private double predictFutureX(RobotData enemy) {
        // Calculate future X coordinate based on enemy data
        double currentX = enemy.scannedX;
        double currentVelocityX = enemy.scannedVelocity * Math.sin(enemy.scannedHeading);
        double futureX = currentX + currentVelocityX * PREDICTION_TIME;

        // Limit futureX to stay within battlefield boundaries
        double minX = HALF_ROBOT_SIZE;
        double maxX = getBattleFieldWidth() - HALF_ROBOT_SIZE;
        return limit(futureX, minX, maxX);
    }

    private double predictFutureY(RobotData enemy) {
        double currentY = enemy.scannedY;
        double currentVelocityY = enemy.scannedVelocity * Math.cos(enemy.scannedHeading);
        double futureY = currentY + currentVelocityY * PREDICTION_TIME;
        double minY = HALF_ROBOT_SIZE;
        double maxY = getBattleFieldHeight() - HALF_ROBOT_SIZE;
        return limit(futureY, minY, maxY);
    }

    private double limit(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void updateTarget() {
        double minDist = Double.POSITIVE_INFINITY;
        RobotData bestTarget = null;

        for (RobotData enemy : enemyMap.values()) {
            double predictedX = predictFutureX(enemy);
            double predictedY = predictFutureY(enemy);
            double dist = distanceTo(predictedX, predictedY);

            if (dist < minDist) {
                minDist = dist;
                bestTarget = enemy;
            }
        }
        target = bestTarget;
    }

    private void updateEnemyMap(ScannedRobotEvent e) {
        // Extract information from the scanned robot event
        String name = e.getName();
        double distance = e.getDistance();
        double bearingRadians = e.getBearingRadians();
        double headingRadians = getHeadingRadians();
        double absBearing = headingRadians + bearingRadians;
        double scannedX = getX() + distance * Math.sin(absBearing);
        double scannedY = getY() + distance * Math.cos(absBearing);
        double velocity = e.getVelocity();
        double heading = e.getHeading();
        double targetBearing = e.getBearing();

        // Update the enemyMap with information about the scanned robot
        if (!enemyMap.containsKey(name)) {
            enemyMap.put(name, new RobotData(name));
        }
        RobotData robotData = enemyMap.get(name);
        robotData.scannedX = scannedX;
        robotData.scannedY = scannedY;
        robotData.scannedVelocity = velocity;
        robotData.scannedHeading = heading;
        robotData.targetBearing = targetBearing;

        // Limit the size of the enemyMap
        if (enemyMap.size() > MAX_ENEMY_MAP_SIZE) {
            // Remove the oldest entry
            Iterator<Map.Entry<String, RobotData>> iterator = enemyMap.entrySet().iterator();
            if (iterator.hasNext()) {
                iterator.next();
                iterator.remove();
            }
        }
    }


    private double bearingTo(double ourHeading, double x, double y) {
        // Calculate the angle to the target position
        double absoluteAngle = Math.atan2(x - getX(), y - getY());

        // Normalize the angle to be relative to our current heading
        double relativeAngle = Utils.normalRelativeAngle(absoluteAngle - ourHeading);

        // Convert the relative angle to bearing (between -180 and 180 degrees)
        double bearing = Math.toDegrees(Utils.normalRelativeAngle(relativeAngle));

        return bearing;
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        updateEnemyMap(e);
        updateTarget(); // Update the target whenever a robot is scanned
        fireGunWhenReady();
    }

    
    private double distanceTo(double x, double y) {
        double dx = x - getX();
        double dy = y - getY();
        return Math.sqrt(dx * dx + dy * dy);
    }


    /**
     * Robot data class to store information about scanned robots.
     */
    private static class RobotData {
        public double targetDistance;
        final String name; // name of the scanned robot
        double scannedX; // x-coordinate when last scanned
        double scannedY; // y-coordinate when last scanned
        double scannedVelocity; // velocity when last scanned
        double scannedHeading; // heading when last scanned
        double targetBearing; // bearing to target

        RobotData(String name) {
            this.name = name;
        }
    }
}
