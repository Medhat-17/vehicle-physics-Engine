#include <Arduino.h>
#include <cmath>
#include <algorithm>

// Define PI for calculations if not already defined (Arduino usually has M_PI)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Vector2D Class ---
// A simple 2D vector class for position, velocity, and force calculations.
// Using 'double' for precision, though on AVR Arduinos, 'double' is typically
// the same as 'float' (32-bit). For more powerful boards (ESP32, Teensy), it's 64-bit.
class Vector2D
{
public:
    double x, y;

    // Default constructor
    Vector2D() : x(0.0), y(0.0) {}
    // Parameterized constructor
    Vector2D(double x_val, double y_val) : x(x_val), y(y_val) {}

    // Vector addition
    Vector2D operator+(const Vector2D &other) const
    {
        return Vector2D(x + other.x, y + other.y);
    }

    // Vector subtraction
    Vector2D operator-(const Vector2D &other) const
    {
        return Vector2D(x - other.x, y - other.y);
    }

    // Scalar multiplication
    Vector2D operator*(double scalar) const
    {
        return Vector2D(x * scalar, y * scalar);
    }

    // Scalar division
    Vector2D operator/(double scalar) const
    {
        if (scalar == 0)
        {
            // Handle division by zero, perhaps return a zero vector or indicate an error
            return Vector2D(0.0, 0.0);
        }
        return Vector2D(x / scalar, y / scalar);
    }

    // Dot product
    double dot(const Vector2D &other) const
    {
        return x * other.x + y * other.y;
    }

    // Magnitude (length) of the vector
    double magnitude() const
    {
        return std::sqrt(x * x + y * y);
    }

    // Squared magnitude (avoids sqrt for comparisons, faster)
    double magnitudeSquared() const
    {
        return x * x + y * y;
    }

    // Normalize the vector (make its magnitude 1)
    Vector2D normalize() const
    {
        double mag = magnitude();
        if (mag > 1e-9)
        { // Avoid division by zero for very small vectors
            return Vector2D(x / mag, y / mag);
        }
        return Vector2D(0.0, 0.0);
    }

    // Rotate the vector by an angle (in radians)
    Vector2D rotate(double angle_rad) const
    {
        double new_x = x * std::cos(angle_rad) - y * std::sin(angle_rad);
        double new_y = x * std::sin(angle_rad) + y * std::cos(angle_rad);
        return Vector2D(new_x, new_y);
    }
};

// --- Vehicle Class ---
class Vehicle
{
public:
    // Physical properties
    double mass;                // kg
    double engineForce;         // N (max force applied by engine)
    double brakeForce;          // N (max braking force)
    double maxSteeringAngle;    // radians (max angle front wheels can turn)
    double wheelbase;           // m (distance between front and rear axles)
    double trackWidth;          // m (distance between left and right wheels) - not used in this simplified steering
    double dragCoefficient;     // dimensionless (for air resistance)
    double rollingResistance;   // N (constant rolling resistance)
    double frictionCoefficient; // dimensionless (coefficient of friction for tires)

    // State variables
    Vector2D position;         // m (x, y coordinates)
    Vector2D velocity;         // m/s (x, y components)
    Vector2D acceleration;     // m/s^2 (x, y components)
    double orientation;        // radians (angle vehicle is facing, 0 = +X axis)
    double steeringAngle;      // radians (current steering input from user)
    double currentEngineForce; // N (actual engine force applied this frame)
    double currentBrakeForce;  // N (actual brake force applied this frame)

    // Constructor
    Vehicle(double m, double engF, double brkF, double maxSA, double wb, double tw, double dragC, double rollR, double fricC)
        : mass(m), engineForce(engF), brakeForce(brkF), maxSteeringAngle(maxSA),
          wheelbase(wb), trackWidth(tw), dragCoefficient(dragC), rollingResistance(rollR),
          frictionCoefficient(fricC),
          position(0.0, 0.0), velocity(0.0, 0.0), acceleration(0.0, 0.0),
          orientation(0.0), steeringAngle(0.0), currentEngineForce(0.0), currentBrakeForce(0.0) {}

    // --- Control Inputs ---

    // Apply engine force (0.0 to 1.0, representing throttle percentage)
    void applyEngine(double throttle)
    {
        currentEngineForce = std::min(std::max(throttle, 0.0), 1.0) * engineForce;
        currentBrakeForce = 0.0; // Release brakes when applying throttle
    }

    // Apply braking force (0.0 to 1.0, representing brake pedal percentage)
    void applyBrake(double brake_input)
    {
        currentBrakeForce = std::min(std::max(brake_input, 0.0), 1.0) * brakeForce;
        currentEngineForce = 0.0; // Release throttle when braking
    }

    // Set steering angle (normalized -1.0 to 1.0, -1.0 for full left, 1.0 for full right)
    void setSteering(double steering_input)
    {
        steeringAngle = std::min(std::max(steering_input, -1.0), 1.0) * maxSteeringAngle;
    }

    // --- Physics Update ---
    // Updates the vehicle's state based on applied forces and time elapsed.
    void update(double deltaTime)
    {
        // 1. Calculate vehicle's forward direction vector
        Vector2D forwardDirection(std::cos(orientation), std::sin(orientation));

        // 2. Calculate forces
        Vector2D totalForce(0.0, 0.0);

        // Engine force (in forward direction)
        totalForce = totalForce + forwardDirection * currentEngineForce;

        // Braking force (opposite to current velocity direction)
        if (velocity.magnitudeSquared() > 1e-6 && currentBrakeForce > 0)
        { // Only apply if moving
            totalForce = totalForce + velocity.normalize() * -currentBrakeForce;
        }

        // Air resistance (drag) - proportional to velocity squared, opposite to velocity
        double speed = velocity.magnitude();
        Vector2D dragForce = velocity.normalize() * (-0.5 * dragCoefficient * speed * speed);
        totalForce = totalForce + dragForce;

        // Rolling resistance - constant force opposite to velocity
        if (speed > 1e-6)
        {
            totalForce = totalForce + velocity.normalize() * -rollingResistance;
        }

        // Tire Friction / Lateral Force (Simplified)
        // This is a very simplified model. A proper model involves slip angles.
        // For now, we'll apply a lateral force that tries to align velocity with orientation.
        Vector2D rightDirection = forwardDirection.rotate(M_PI / 2.0); // Perpendicular to forward
        double lateralVelocity = velocity.dot(rightDirection);
        Vector2D lateralForce = rightDirection * (-lateralVelocity * frictionCoefficient * mass); // Proportional to lateral velocity and friction

        // Limit lateral force by available friction (simplified friction circle)
        double maxLateralForce = frictionCoefficient * mass * 9.81; // Max friction based on normal force (assuming gravity ~9.81 m/s^2)
        if (lateralForce.magnitude() > maxLateralForce)
        {
            lateralForce = lateralForce.normalize() * maxLateralForce;
        }
        totalForce = totalForce + lateralForce;

        // 3. Calculate acceleration (F = ma => a = F/m)
        acceleration = totalForce / mass;

        // 4. Update velocity (Euler integration)
        velocity = velocity + acceleration * deltaTime;

        // 5. Update position (Euler integration)
        position = position + velocity * deltaTime;

        // 6. Update orientation (Steering Dynamics - Ackermann Steering Simplified)
        // This simplified model assumes the vehicle rotates around an instantaneous center of rotation.
        // The turning radius is dependent on wheelbase and steering angle.
        if (std::abs(steeringAngle) > 1e-6)
        { // Only rotate if steering
            // Calculate turning radius (simplified for small angles)
            // Angular velocity = V / R
            // R = L / tan(delta) where L is wheelbase, delta is steering angle
            double angularVelocity = (speed / wheelbase) * std::tan(steeringAngle);
            orientation += angularVelocity * deltaTime;
        }
        else
        {
            // If not steering, align orientation with velocity if moving
            if (speed > 1e-6)
            {
                orientation = std::atan2(velocity.y, velocity.x);
            }
        }

        // Normalize orientation to be within [0, 2*PI)
        orientation = std::fmod(orientation, 2.0 * M_PI);
        if (orientation < 0)
        {
            orientation += 2.0 * M_PI;
        }

        // Very basic collision detection with a "wall" at x=100 and x=-100
        // This is a placeholder; a real engine would use bounding boxes/spheres.
        if (position.x > 100.0 || position.x < -100.0 || position.y > 100.0 || position.y < -100.0)
        {
            Serial.println("Collision detected with boundary! Resetting position and velocity.");
            position = Vector2D(0.0, 0.0);
            velocity = Vector2D(0.0, 0.0);
            acceleration = Vector2D(0.0, 0.0);
            orientation = 0.0;
        }
    }

    // Display current state for debugging
    void printState() const
    {
        Serial.print("Pos: (");
        Serial.print(position.x);
        Serial.print(", ");
        Serial.print(position.y);
        Serial.print(") ");
        Serial.print("Vel: (");
        Serial.print(velocity.x);
        Serial.print(", ");
        Serial.print(velocity.y);
        Serial.print(") ");
        Serial.print("Speed: ");
        Serial.print(velocity.magnitude());
        Serial.print(" m/s ");
        Serial.print("Acc: (");
        Serial.print(acceleration.x);
        Serial.print(", ");
        Serial.print(acceleration.y);
        Serial.print(") ");
        Serial.print("Orient: ");
        Serial.print(orientation * 180.0 / M_PI);
        Serial.print(" deg ");
        Serial.print("Steer: ");
        Serial.print(steeringAngle * 180.0 / M_PI);
        Serial.print(" deg ");
        Serial.print("Engine: ");
        Serial.print(currentEngineForce);
        Serial.print(" N ");
        Serial.print("Brake: ");
        Serial.print(currentBrakeForce);
        Serial.println(" N ");
    }
};

// Global vehicle instance
Vehicle car(1200.0, 5000.0, 8000.0, 30.0 * M_PI / 180.0, 2.8, 1.6, 0.4, 300.0, 0.8);

// Time variables for deltaTime calculation
unsigned long previousMillis = 0;
const long printInterval = 500; // Print state every 500 milliseconds (0.5 seconds)
unsigned long previousPrintMillis = 0;

// Simulation time counter
double simulationTime = 0.0;

// --- Arduino Setup Function ---
void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    while (!Serial)
    {
        ; // Wait for serial port to connect. Needed for native USB port only
    }

    Serial.println("Starting Vehicle Physics Simulation (Arduino)...");
    Serial.println("Initial State:");
    car.printState();
    Serial.println("-------------------------------------");

    previousMillis = millis();      // Initialize previousMillis for deltaTime calculation
    previousPrintMillis = millis(); // Initialize previousPrintMillis for print interval
}

// --- Arduino Loop Function ---
void loop()
{
    unsigned long currentMillis = millis();
    double deltaTime = (currentMillis - previousMillis) / 1000.0; // Convert milliseconds to seconds
    previousMillis = currentMillis;

    // Only update if a significant amount of time has passed to avoid tiny steps
    // and potential floating point issues on very fast loops.
    // A minimum deltaTime can also prevent division by zero if millis() returns same value.
    if (deltaTime < 0.001)
    { // If less than 1ms has passed, skip update
        return;
    }

    simulationTime += deltaTime;

    // --- Apply Control Inputs (Example Scenario) ---
    // This part mimics the time-based scenario from the original C++ main function.
    // In a real Arduino application, these would come from sensor readings (e.g., analogRead for throttle/steering).
    if (simulationTime < 5.0)
    {
        // Accelerate forward
        car.applyEngine(1.0); // Full throttle
        car.setSteering(0.0); // Straight
    }
    else if (simulationTime < 10.0)
    {
        // Turn left while maintaining some throttle
        car.applyEngine(0.5);  // Half throttle
        car.setSteering(-0.8); // Turn left
    }
    else if (simulationTime < 15.0)
    {
        // Brake
        car.applyBrake(1.0);  // Full brake
        car.setSteering(0.0); // Straight
    }
    else if (simulationTime < 20.0)
    {
        // Coasting or light acceleration
        car.applyEngine(0.2);
        car.setSteering(0.0);
    }
    else
    {
        // Simulation finished, stop applying forces
        car.applyEngine(0.0);
        car.applyBrake(0.0);
        car.setSteering(0.0);
        // Optionally, you could add a delay or put the Arduino to sleep here
        // to prevent it from constantly running the physics if the simulation is 'over'.
    }

    // Update the vehicle's state
    car.update(deltaTime);

    // Print state every `printInterval` milliseconds for readability
    if (currentMillis - previousPrintMillis >= printInterval)
    {
        Serial.print("Time: ");
        Serial.print(simulationTime);
        Serial.print("s | ");
        car.printState();
        previousPrintMillis = currentMillis;
    }
}
