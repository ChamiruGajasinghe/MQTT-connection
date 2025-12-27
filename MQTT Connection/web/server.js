// MQTT IoT Monitoring Web Server with TimescaleDB Integration
require('dotenv').config(); // Load environment variables from .env file
const express = require('express');
const path = require('path');
const mqtt = require('mqtt');
const { Pool } = require('pg');

const app = express();
const PORT = process.env.PORT || 3000;

// TimescaleDB connection configuration
const dbConfig = {
    host: process.env.TIMESCALEDB_HOST || 'localhost',
    port: parseInt(process.env.TIMESCALEDB_PORT || '5432'),
    database: process.env.TIMESCALEDB_DATABASE || 'iot_monitoring',
    user: process.env.TIMESCALEDB_USER || 'postgres',
    password: process.env.TIMESCALEDB_PASSWORD || 'postgres',
    max: 20, // Maximum number of clients in the pool
    idleTimeoutMillis: 30000,
    connectionTimeoutMillis: 2000,
};

// Create PostgreSQL connection pool
const pool = new Pool(dbConfig);

// Test database connection
pool.on('connect', () => {
    console.log('✓ Connected to TimescaleDB');
});

pool.on('error', (err) => {
    console.error('✗ TimescaleDB connection error:', err);
    // Attempt to reconnect after 5 seconds
    setTimeout(() => {
        console.log('Attempting to reconnect to TimescaleDB...');
    }, 5000);
});

// Test database connection on startup
(async () => {
    try {
        const result = await pool.query('SELECT NOW()');
        console.log('✓ TimescaleDB connection verified:', result.rows[0].now);
    } catch (error) {
        console.error('✗ Failed to connect to TimescaleDB:', error.message);
        console.error('  Please check your database configuration in .env file');
        console.error('  Make sure PostgreSQL and TimescaleDB are running');
    }
})();

// MQTT Broker configuration
const MQTT_BROKER = process.env.MQTT_BROKER || 'wss://broker.hivemq.com:8884/mqtt';
const MQTT_TOPICS = [
    'esp32/sensor/voltage',
    'esp32/sensor/current',
    'esp32/sensor2/voltage',
    'esp32/sensor2/current',
    'esp32/light/intensity',
    'esp32/led/status',
    'esp32/led2/status',
    'esp32/led4/status',
    'esp32/emergency/status',
    'esp32/powercut/status',
    'esp32/history/powercut',
    'esp32/command/status'
];

// Connect to MQTT broker
let mqttClient = null;
let lastSensorReading = {
    battery_voltage: null,
    battery_current: null,
    main_voltage: null,
    main_current: null,
    light_intensity: null,
    led_status: null,
    led2_status: null,
    led4_status: null,
    emergency_light_status: null,
    power_cut_status: 'NORMAL'
};

function connectMQTT() {
    console.log('Connecting to MQTT broker:', MQTT_BROKER);
    
    mqttClient = mqtt.connect(MQTT_BROKER, {
        clientId: 'WebServer_' + Math.random().toString(16).substr(2, 8),
        clean: true,
        reconnectPeriod: 1000,
    });

    mqttClient.on('connect', () => {
        console.log('✓ Connected to MQTT broker');
        // Subscribe to all topics
        MQTT_TOPICS.forEach(topic => {
            mqttClient.subscribe(topic, (err) => {
                if (err) {
                    console.error(`✗ Failed to subscribe to ${topic}:`, err);
                } else {
                    console.log(`✓ Subscribed to ${topic}`);
                }
            });
        });
    });

    mqttClient.on('message', async (topic, message) => {
        const msg = message.toString();
        const timestamp = new Date();
        
        try {
            // Update last sensor reading
            switch (topic) {
                case 'esp32/sensor/voltage':
                    lastSensorReading.battery_voltage = parseFloat(msg);
                    break;
                case 'esp32/sensor/current':
                    lastSensorReading.battery_current = parseFloat(msg);
                    break;
                case 'esp32/sensor2/voltage':
                    lastSensorReading.main_voltage = parseFloat(msg);
                    break;
                case 'esp32/sensor2/current':
                    lastSensorReading.main_current = parseFloat(msg);
                    break;
                case 'esp32/light/intensity':
                    lastSensorReading.light_intensity = parseFloat(msg);
                    break;
                case 'esp32/led/status':
                    lastSensorReading.led_status = msg;
                    break;
                case 'esp32/led2/status':
                    lastSensorReading.led2_status = msg;
                    break;
                case 'esp32/led4/status':
                    lastSensorReading.led4_status = msg;
                    break;
                case 'esp32/emergency/status':
                    lastSensorReading.emergency_light_status = msg;
                    break;
                case 'esp32/powercut/status':
                    lastSensorReading.power_cut_status = msg;
                    break;
            }

            // Store sensor readings every 2 seconds (when we have all data)
            if (lastSensorReading.battery_voltage !== null && 
                lastSensorReading.main_voltage !== null) {
                await storeSensorReading(timestamp);
            }

            // Handle power cut history
            if (topic === 'esp32/history/powercut') {
                await storePowerCutHistory(msg, timestamp);
            }

            // Store command logs
            if (topic === 'esp32/command/status' && msg !== 'CLEAR_LOG') {
                await storeCommandLog(topic, msg, timestamp);
            }
        } catch (error) {
            console.error(`Error processing MQTT message from ${topic}:`, error);
        }
    });

    mqttClient.on('error', (error) => {
        console.error('MQTT error:', error);
    });

    mqttClient.on('close', () => {
        console.log('MQTT connection closed');
    });

    mqttClient.on('reconnect', () => {
        console.log('Reconnecting to MQTT broker...');
    });
}

// Store sensor reading in TimescaleDB
async function storeSensorReading(timestamp) {
    const deviceId = process.env.DEVICE_ID || 'ESP32_001';
    
    // Calculate power values
    const batteryPower = lastSensorReading.battery_voltage * (lastSensorReading.battery_current / 1000.0);
    const mainPower = lastSensorReading.main_voltage * (lastSensorReading.main_current / 1000.0);

    const query = `
        INSERT INTO sensor_readings (
            time, device_id,
            battery_voltage, battery_current, battery_power,
            main_voltage, main_current, main_power,
            light_intensity,
            led_status, led2_status, led4_status, emergency_light_status,
            power_cut_status
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14)
        ON CONFLICT DO NOTHING
    `;

    try {
        await pool.query(query, [
            timestamp,
            deviceId,
            lastSensorReading.battery_voltage,
            lastSensorReading.battery_current,
            batteryPower,
            lastSensorReading.main_voltage,
            lastSensorReading.main_current,
            mainPower,
            lastSensorReading.light_intensity,
            lastSensorReading.led_status,
            lastSensorReading.led2_status,
            lastSensorReading.led4_status,
            lastSensorReading.emergency_light_status,
            lastSensorReading.power_cut_status
        ]);
    } catch (error) {
        console.error('Error storing sensor reading:', error);
    }
}

// Store power cut history
async function storePowerCutHistory(jsonData, timestamp) {
    try {
        const data = JSON.parse(jsonData);
        const deviceId = process.env.DEVICE_ID || 'ESP32_001';
        
        const query = `
            INSERT INTO power_cut_events (
                device_id, start_time, end_time, duration_ms,
                start_voltage, end_voltage, voltage_drop, energy_consumed_mwh
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
        `;

        const endTime = new Date(timestamp.getTime() - data.duration);
        
        await pool.query(query, [
            deviceId,
            endTime,
            timestamp,
            data.duration,
            data.startV,
            data.endV,
            data.drop,
            data.energy
        ]);
    } catch (error) {
        console.error('Error storing power cut history:', error);
    }
}

// Store command log
async function storeCommandLog(topic, message, timestamp) {
    const deviceId = process.env.DEVICE_ID || 'ESP32_001';
    
    const query = `
        INSERT INTO command_logs (device_id, time, command_type, command_value, topic, message)
        VALUES ($1, $2, $3, $4, $5, $6)
    `;

    try {
        // Extract command type from topic
        const commandType = topic.split('/').pop().toUpperCase();
        
        await pool.query(query, [
            deviceId,
            timestamp,
            commandType,
            message.substring(0, 50), // Limit command value length
            topic,
            message
        ]);
    } catch (error) {
        console.error('Error storing command log:', error);
    }
}

// Middleware
app.use(express.static(__dirname));
app.use(express.json());

// Serve main dashboard
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'dashboard.html'));
});

app.get('/dashboard', (req, res) => {
    res.sendFile(path.join(__dirname, 'dashboard.html'));
});

app.get('/dashboard.html', (req, res) => {
    res.sendFile(path.join(__dirname, 'dashboard.html'));
});

// Serve history page
app.get('/history', (req, res) => {
    res.sendFile(path.join(__dirname, 'history.html'));
});

app.get('/history.html', (req, res) => {
    res.sendFile(path.join(__dirname, 'history.html'));
});

// Serve global control page
app.get('/global-control', (req, res) => {
    res.sendFile(path.join(__dirname, 'global-control.html'));
});

app.get('/global-control.html', (req, res) => {
    res.sendFile(path.join(__dirname, 'global-control.html'));
});

// Health check endpoint
app.get('/api/health', async (req, res) => {
    let dbStatus = 'disconnected';
    try {
        const result = await pool.query('SELECT NOW()');
        dbStatus = 'connected';
    } catch (error) {
        console.error('Database health check failed:', error);
    }

    res.json({ 
        status: 'online',
        database: dbStatus,
        mqtt: mqttClient && mqttClient.connected ? 'connected' : 'disconnected',
        timestamp: new Date().toISOString(),
        uptime: process.uptime()
    });
});

// API: Get sensor readings (time-series data)
app.get('/api/sensor-readings', async (req, res) => {
    try {
        const { start_time, end_time, limit = 1000 } = req.query;
        const deviceId = process.env.DEVICE_ID || 'ESP32_001';
        
        let query = `
            SELECT time, battery_voltage, battery_current, battery_power,
                   main_voltage, main_current, main_power, light_intensity,
                   led_status, led2_status, led4_status, emergency_light_status,
                   power_cut_status
            FROM sensor_readings
            WHERE device_id = $1
        `;
        const params = [deviceId];
        let paramIndex = 2;

        if (start_time) {
            query += ` AND time >= $${paramIndex}`;
            params.push(new Date(start_time));
            paramIndex++;
        }

        if (end_time) {
            query += ` AND time <= $${paramIndex}`;
            params.push(new Date(end_time));
            paramIndex++;
        }

        query += ` ORDER BY time DESC LIMIT $${paramIndex}`;
        params.push(parseInt(limit));

        const result = await pool.query(query, params);
        res.json(result.rows);
    } catch (error) {
        console.error('Error fetching sensor readings:', error);
        res.status(500).json({ error: 'Failed to fetch sensor readings' });
    }
});

// API: Get power cut events
app.get('/api/power-cut-events', async (req, res) => {
    try {
        const { start_time, end_time, limit = 100 } = req.query;
        const deviceId = process.env.DEVICE_ID || 'ESP32_001';
        
        let query = `
            SELECT id, start_time, end_time, duration_ms,
                   start_voltage, end_voltage, voltage_drop, energy_consumed_mwh
            FROM power_cut_events
            WHERE device_id = $1
        `;
        const params = [deviceId];
        let paramIndex = 2;

        if (start_time) {
            query += ` AND start_time >= $${paramIndex}`;
            params.push(new Date(start_time));
            paramIndex++;
        }

        if (end_time) {
            query += ` AND start_time <= $${paramIndex}`;
            params.push(new Date(end_time));
            paramIndex++;
        }

        query += ` ORDER BY start_time DESC LIMIT $${paramIndex}`;
        params.push(parseInt(limit));

        const result = await pool.query(query, params);
        res.json(result.rows);
    } catch (error) {
        console.error('Error fetching power cut events:', error);
        res.status(500).json({ error: 'Failed to fetch power cut events' });
    }
});

// API: Get command logs
app.get('/api/command-logs', async (req, res) => {
    try {
        const { start_time, end_time, limit = 100 } = req.query;
        const deviceId = process.env.DEVICE_ID || 'ESP32_001';
        
        let query = `
            SELECT id, time, command_type, command_value, topic, message
            FROM command_logs
            WHERE device_id = $1
        `;
        const params = [deviceId];
        let paramIndex = 2;

        if (start_time) {
            query += ` AND time >= $${paramIndex}`;
            params.push(new Date(start_time));
            paramIndex++;
        }

        if (end_time) {
            query += ` AND time <= $${paramIndex}`;
            params.push(new Date(end_time));
            paramIndex++;
        }

        query += ` ORDER BY time DESC LIMIT $${paramIndex}`;
        params.push(parseInt(limit));

        const result = await pool.query(query, params);
        res.json(result.rows);
    } catch (error) {
        console.error('Error fetching command logs:', error);
        res.status(500).json({ error: 'Failed to fetch command logs' });
    }
});

// API: Get aggregated statistics
app.get('/api/statistics', async (req, res) => {
    try {
        const { hours = 24 } = req.query;
        const deviceId = process.env.DEVICE_ID || 'ESP32_001';
        
        const query = `
            SELECT 
                AVG(battery_voltage) as avg_battery_voltage,
                AVG(battery_current) as avg_battery_current,
                AVG(main_voltage) as avg_main_voltage,
                AVG(main_current) as avg_main_current,
                AVG(light_intensity) as avg_light_intensity,
                COUNT(*) as total_readings,
                MIN(time) as first_reading,
                MAX(time) as last_reading
            FROM sensor_readings
            WHERE device_id = $1 
            AND time >= NOW() - INTERVAL '${parseInt(hours)} hours'
        `;

        const result = await pool.query(query, [deviceId]);
        res.json(result.rows[0] || {});
    } catch (error) {
        console.error('Error fetching statistics:', error);
        res.status(500).json({ error: 'Failed to fetch statistics' });
    }
});

// Start server
app.listen(PORT, '0.0.0.0', async () => {
    console.log('=============================================');
    console.log('🌐 IoT Monitoring System - GLOBAL ACCESS');
    console.log('=============================================');
    console.log(`📡 Local Access:    http://localhost:${PORT}`);
    console.log(`🌍 Network Access:  http://[YOUR_IP]:${PORT}`);
    console.log(`📊 Dashboard:       http://localhost:${PORT}/dashboard.html`);
    console.log(`📈 History:         http://localhost:${PORT}/history.html`);
    console.log(`🎛️  Control:         http://localhost:${PORT}/global-control.html`);
    console.log(`🔌 MQTT Broker:     ${MQTT_BROKER}`);
    console.log(`🗄️  TimescaleDB:     ${dbConfig.host}:${dbConfig.port}/${dbConfig.database}`);
    console.log('=============================================');
    
    // Verify database connection before starting
    try {
        const dbCheck = await pool.query('SELECT NOW()');
        console.log('✓ Database connection active');
    } catch (error) {
        console.error('⚠ Warning: Database connection failed:', error.message);
        console.error('  Server will continue, but data will not be stored');
        console.error('  Please check your database configuration');
    }
    
    // Connect to MQTT broker
    connectMQTT();
    
    console.log('💡 For internet access:');
    console.log('   1. Get your public IP from: https://whatismyipaddress.com');
    console.log('   2. Forward port', PORT, 'in your router settings');
    console.log('   3. Access via: http://[PUBLIC_IP]:' + PORT);
    console.log('   OR use a service like: ngrok, localtunnel, or deploy to cloud');
    console.log('=============================================');
});

// Graceful shutdown
process.on('SIGTERM', async () => {
    console.log('SIGTERM signal received: closing HTTP server');
    if (mqttClient) {
        mqttClient.end();
    }
    await pool.end();
    process.exit(0);
});

process.on('SIGINT', async () => {
    console.log('SIGINT signal received: closing HTTP server');
    if (mqttClient) {
        mqttClient.end();
    }
    await pool.end();
    process.exit(0);
});
