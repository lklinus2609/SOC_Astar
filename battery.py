import math

class LiFePO4Battery:
    def __init__(self, capacity, initial_charge=1.0):
        self.capacity = capacity  # Total capacity in watt-hours
        self.charge = initial_charge * capacity  # Current charge in watt-hours
        self.discharge_efficiency = 0.95  # Discharge efficiency
        self.regen_efficiency = 0.70  # Regenerative efficiency (for downhill)
        self.robot_mass = 50.0  # kg - mass of the robot
        self.safety_threshold = 0.1  # 10% of capacity as safety margin
        
        # Voltage-SoC curve (characteristic flat discharge of LiFePO4)
        self.voltage_curve = {
            1.0: 13.5,   # 100% SOC
            0.9: 13.3,
            0.8: 13.2,
            0.7: 13.1,
            0.6: 13.0,
            0.5: 12.9,   # Very flat in the middle range
            0.4: 12.8,
            0.3: 12.7,
            0.2: 12.5,
            0.1: 12.0,
            0.0: 10.0    # Empty
        }
        
        # Non-linear efficiency curve
        self.efficiency_curve = {
            1.0: 0.95,  # Efficiency at full charge
            0.8: 0.92,
            0.6: 0.90,
            0.4: 0.85,
            0.2: 0.80,
            0.0: 0.75   # Efficiency at near-empty
        }
        
    def get_state_of_charge(self):
        """Return the current state of charge as a ratio (0-1)"""
        return self.charge / self.capacity
    
    def get_efficiency_at_soc(self, soc):
        """Get discharge efficiency at a given state of charge using interpolation"""
        # Find closest SOC points in efficiency curve
        keys = sorted(self.efficiency_curve.keys())
        
        if soc <= keys[0]:
            return self.efficiency_curve[keys[0]]
        
        if soc >= keys[-1]:
            return self.efficiency_curve[keys[-1]]
        
        # Linear interpolation between closest values
        for i in range(len(keys) - 1):
            if keys[i] <= soc <= keys[i+1]:
                t = (soc - keys[i]) / (keys[i+1] - keys[i])
                return (1 - t) * self.efficiency_curve[keys[i]] + t * self.efficiency_curve[keys[i+1]]
        
        # Fallback to default efficiency
        return self.discharge_efficiency
    
    def calculate_consumption(self, current_elevation, next_elevation, distance, terrain_type=None):
        """
        Calculate energy consumption based on physical model
        
        Args:
            current_elevation: Elevation at current position
            next_elevation: Elevation at next position
            distance: Distance between positions
            terrain_type: Optional terrain type information (kept for backward compatibility but not used)
            
        Returns:
            Energy consumption in watt-hours
        """
        # Base energy consumption per unit distance (flat terrain)
        base_consumption = 1.0 * distance
        
        # Calculate elevation difference and slope
        elevation_diff = next_elevation - current_elevation
        slope_rad = math.atan2(elevation_diff, distance) if distance > 0 else 0
        
        # Calculate gravitational force component
        gravitational_force = self.robot_mass * 9.81 * math.sin(slope_rad)
        
        # Get current efficiency based on state of charge
        current_soc = self.get_state_of_charge()
        current_efficiency = self.get_efficiency_at_soc(current_soc)
        
        if elevation_diff > 0:  # Uphill
            # Energy required to overcome gravitational force - purely physics-based
            work = gravitational_force * distance
            # Add base consumption and apply efficiency
            total_consumption = (base_consumption + work) / current_efficiency
        else:  # Downhill or flat
            # Potential energy recovery from downhill (if any) - purely physics-based
            recovery = max(0, -gravitational_force * distance * self.regen_efficiency)
            # Net consumption (with potential regeneration)
            total_consumption = max(0, base_consumption - recovery)
        
        return total_consumption
    
    def update_state(self, energy_consumed):
        """Update battery state after consuming energy"""
        self.charge -= energy_consumed
        # Ensure charge stays within bounds
        self.charge = max(0.0, min(self.capacity, self.charge))
        return self.charge > 0  # Return True if battery still has charge
    
    def can_traverse(self, current_elevation, next_elevation, distance, terrain_type=None):
        """Check if battery has enough charge to traverse segment, with safety margin"""
        consumption = self.calculate_consumption(current_elevation, next_elevation, distance, terrain_type)
        remaining_charge = self.charge - consumption
        
        # Check against safety threshold
        return remaining_charge >= self.capacity * self.safety_threshold
    
    def clone(self):
        """Create a copy of the battery with the same state"""
        battery_copy = LiFePO4Battery(self.capacity, 0)
        battery_copy.charge = self.charge
        battery_copy.robot_mass = self.robot_mass
        battery_copy.discharge_efficiency = self.discharge_efficiency
        battery_copy.regen_efficiency = self.regen_efficiency
        battery_copy.safety_threshold = self.safety_threshold
        return battery_copy