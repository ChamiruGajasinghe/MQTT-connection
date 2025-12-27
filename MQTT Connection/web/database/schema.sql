-- TimescaleDB Schema for IoT Monitoring System
-- This script creates all necessary tables and converts them to hypertables

-- Enable TimescaleDB extension
CREATE EXTENSION IF NOT EXISTS timescaledb;

-- ============================================
-- Sensor Readings Table (Time-Series Data)
-- ============================================
CREATE TABLE IF NOT EXISTS sensor_readings (
    time TIMESTAMPTZ NOT NULL,
    device_id VARCHAR(50) NOT NULL,
    battery_voltage NUMERIC(10, 3),
    battery_current NUMERIC(10, 3),
    battery_power NUMERIC(10, 3),
    main_voltage NUMERIC(10, 3),
    main_current NUMERIC(10, 3),
    main_power NUMERIC(10, 3),
    light_intensity NUMERIC(5, 1),
    led_status VARCHAR(10),
    led2_status VARCHAR(10),
    led4_status VARCHAR(10),
    emergency_light_status VARCHAR(10),
    power_cut_status VARCHAR(20) DEFAULT 'NORMAL'
);

-- Create hypertable for sensor_readings (partitioned by time)
SELECT create_hypertable('sensor_readings', 'time', if_not_exists => TRUE);

-- Create index on device_id for faster queries
CREATE INDEX IF NOT EXISTS idx_sensor_readings_device_id ON sensor_readings(device_id);
CREATE INDEX IF NOT EXISTS idx_sensor_readings_time_device ON sensor_readings(time DESC, device_id);

-- ============================================
-- Power Cut Events Table
-- ============================================
CREATE TABLE IF NOT EXISTS power_cut_events (
    id SERIAL PRIMARY KEY,
    device_id VARCHAR(50) NOT NULL,
    start_time TIMESTAMPTZ NOT NULL,
    end_time TIMESTAMPTZ NOT NULL,
    duration_ms BIGINT NOT NULL,
    start_voltage NUMERIC(10, 3),
    end_voltage NUMERIC(10, 3),
    voltage_drop NUMERIC(10, 3),
    energy_consumed_mwh NUMERIC(10, 2),
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Create hypertable for power_cut_events
SELECT create_hypertable('power_cut_events', 'start_time', if_not_exists => TRUE);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_power_cut_events_device_id ON power_cut_events(device_id);
CREATE INDEX IF NOT EXISTS idx_power_cut_events_start_time ON power_cut_events(start_time DESC);

-- ============================================
-- Command Logs Table
-- ============================================
CREATE TABLE IF NOT EXISTS command_logs (
    id SERIAL PRIMARY KEY,
    device_id VARCHAR(50) NOT NULL,
    time TIMESTAMPTZ NOT NULL,
    command_type VARCHAR(50),
    command_value VARCHAR(255),
    topic VARCHAR(255),
    message TEXT,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Create hypertable for command_logs
SELECT create_hypertable('command_logs', 'time', if_not_exists => TRUE);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_command_logs_device_id ON command_logs(device_id);
CREATE INDEX IF NOT EXISTS idx_command_logs_time ON command_logs(time DESC);
CREATE INDEX IF NOT EXISTS idx_command_logs_command_type ON command_logs(command_type);

-- ============================================
-- Retention Policy (Optional - Clean old data)
-- ============================================
-- Uncomment to enable automatic data retention (keeps last 90 days)
-- SELECT add_retention_policy('sensor_readings', INTERVAL '90 days', if_not_exists => TRUE);
-- SELECT add_retention_policy('power_cut_events', INTERVAL '365 days', if_not_exists => TRUE);
-- SELECT add_retention_policy('command_logs', INTERVAL '90 days', if_not_exists => TRUE);

-- ============================================
-- Continuous Aggregates (Optional - Pre-computed statistics)
-- ============================================
-- Create continuous aggregate for hourly statistics
CREATE MATERIALIZED VIEW IF NOT EXISTS sensor_readings_hourly
WITH (timescaledb.continuous) AS
SELECT
    time_bucket('1 hour', time) AS bucket,
    device_id,
    AVG(battery_voltage) AS avg_battery_voltage,
    AVG(battery_current) AS avg_battery_current,
    AVG(main_voltage) AS avg_main_voltage,
    AVG(main_current) AS avg_main_current,
    AVG(light_intensity) AS avg_light_intensity,
    COUNT(*) AS reading_count
FROM sensor_readings
GROUP BY bucket, device_id;

-- Add refresh policy for continuous aggregate (refresh every hour)
SELECT add_continuous_aggregate_policy('sensor_readings_hourly',
    start_offset => INTERVAL '3 hours',
    end_offset => INTERVAL '1 hour',
    schedule_interval => INTERVAL '1 hour',
    if_not_exists => TRUE);

-- ============================================
-- Grant Permissions (Adjust as needed)
-- ============================================
-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO your_user;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO your_user;

