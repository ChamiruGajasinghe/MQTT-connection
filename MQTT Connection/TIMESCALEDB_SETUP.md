# TimescaleDB Setup Guide

Complete guide for setting up TimescaleDB for the IoT Monitoring System, both locally and in the cloud.

---

## 📋 Table of Contents

1. [Local Setup](#local-setup)
2. [Cloud Setup](#cloud-setup)
3. [Database Schema](#database-schema)
4. [Troubleshooting](#troubleshooting)

---

## 🏠 Local Setup

### Prerequisites

- **PostgreSQL 12+** installed on your system
- **TimescaleDB extension** installed
- **Node.js 14+** and npm
- **psql** command-line tool (comes with PostgreSQL)

### Step 1: Install PostgreSQL and TimescaleDB

#### Linux (Ubuntu/Debian)

```bash
# Add PostgreSQL repository
sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list.d/pgdg.list'
wget --quiet -O - https://www.postgresql.org/media/keys/ACCC4CF8.asc | sudo apt-key add -
sudo apt-get update

# Install PostgreSQL
sudo apt-get install postgresql postgresql-contrib

# Install TimescaleDB
sudo sh -c "echo 'deb https://packagecloud.io/timescale/timescaledb/ubuntu/ $(lsb_release -c -s) main' > /etc/apt/sources.list.d/timescaledb.list"
wget --quiet -O - https://packagecloud.io/timescale/timescaledb/gpgkey | sudo apt-key add -
sudo apt-get update
sudo apt-get install timescaledb-2-postgresql-14  # Adjust version to match your PostgreSQL version

# Configure TimescaleDB
sudo timescaledb-tune
```

#### macOS

```bash
# Using Homebrew
brew install postgresql@14
brew install timescaledb

# Link PostgreSQL
brew link postgresql@14

# Configure TimescaleDB
timescaledb-tune
```

#### Windows

1. Download PostgreSQL from: https://www.postgresql.org/download/windows/
2. Download TimescaleDB from: https://docs.timescale.com/install/latest/self-hosted/
3. Follow the installation wizard
4. Make sure to add PostgreSQL to your PATH

### Step 2: Start PostgreSQL Service

#### Linux

```bash
sudo systemctl start postgresql
sudo systemctl enable postgresql  # Enable auto-start on boot
```

#### macOS

```bash
brew services start postgresql@14
```

#### Windows

PostgreSQL service should start automatically after installation. You can manage it from Services.

### Step 3: Create Database and User

```bash
# Connect to PostgreSQL as superuser
sudo -u postgres psql  # Linux
# OR
psql -U postgres  # macOS/Windows (if password is set)

# Create database
CREATE DATABASE iot_monitoring;

# Create user (optional, or use existing postgres user)
CREATE USER iot_user WITH PASSWORD 'your_secure_password';
GRANT ALL PRIVILEGES ON DATABASE iot_monitoring TO iot_user;

# Exit psql
\q
```

### Step 4: Initialize Database Schema

Navigate to the web directory:

```bash
cd "MQTT Connection/web"
```

#### Linux/macOS

```bash
# Make script executable
chmod +x database/init-db.sh

# Run initialization script
./database/init-db.sh
```

#### Windows (PowerShell)

```powershell
# Run initialization script
.\database\init-db.ps1
```

#### Manual Method

If scripts don't work, you can run the schema manually:

```bash
# Set password (Linux/macOS)
export PGPASSWORD=postgres

# Run schema
psql -h localhost -U postgres -d iot_monitoring -f database/schema.sql
```

### Step 5: Configure Environment Variables

1. Copy the example environment file:

```bash
cp env.example .env
```

2. Edit `.env` file with your database credentials:

```env
TIMESCALEDB_HOST=localhost
TIMESCALEDB_PORT=5432
TIMESCALEDB_DATABASE=iot_monitoring
TIMESCALEDB_USER=postgres
TIMESCALEDB_PASSWORD=postgres
DEVICE_ID=ESP32_001
PORT=3000
MQTT_BROKER=wss://broker.hivemq.com:8884/mqtt
```

### Step 6: Install Node.js Dependencies

```bash
# Install dotenv for environment variable support
npm install dotenv

# Or reinstall all dependencies
npm install
```

### Step 7: Start the Server

```bash
npm start
```

You should see:

```
✓ Connected to TimescaleDB
✓ Connected to MQTT broker
Server running on http://localhost:3000
```

---

## ☁️ Cloud Setup

### Option 1: Timescale Cloud (Recommended)

Timescale Cloud is a fully managed TimescaleDB service.

#### Step 1: Create Timescale Cloud Account

1. Go to https://www.timescale.com/cloud
2. Sign up for a free account (14-day trial)
3. Create a new service

#### Step 2: Get Connection Details

After creating a service, you'll receive:
- **Host**: `xxxxx.cloud.timescale.com`
- **Port**: `5432`
- **Database**: `tsdb`
- **Username**: `tsdbadmin`
- **Password**: (provided in dashboard)

#### Step 3: Configure Environment Variables

Update your `.env` file:

```env
TIMESCALEDB_HOST=xxxxx.cloud.timescale.com
TIMESCALEDB_PORT=5432
TIMESCALEDB_DATABASE=tsdb
TIMESCALEDB_USER=tsdbadmin
TIMESCALEDB_PASSWORD=your_cloud_password
DEVICE_ID=ESP32_001
PORT=3000
MQTT_BROKER=wss://broker.hivemq.com:8884/mqtt
```

#### Step 4: Initialize Schema on Cloud

Connect to your cloud database and run the schema:

```bash
# Using psql
psql -h xxxxx.cloud.timescale.com -U tsdbadmin -d tsdb -f database/schema.sql
```

Or use the initialization script with cloud credentials:

```bash
# Update .env first, then:
./database/init-db.sh
```

### Option 2: AWS RDS with TimescaleDB

#### Step 1: Launch RDS Instance

1. Go to AWS RDS Console
2. Create a new PostgreSQL instance
3. Choose PostgreSQL 14 or higher
4. Configure instance settings
5. Note the endpoint and credentials

#### Step 2: Install TimescaleDB Extension

Connect to your RDS instance:

```sql
-- Connect via psql or AWS RDS Query Editor
CREATE EXTENSION IF NOT EXISTS timescaledb;
```

#### Step 3: Configure Environment Variables

```env
TIMESCALEDB_HOST=your-rds-endpoint.region.rds.amazonaws.com
TIMESCALEDB_PORT=5432
TIMESCALEDB_DATABASE=iot_monitoring
TIMESCALEDB_USER=your_username
TIMESCALEDB_PASSWORD=your_password
```

### Option 3: DigitalOcean Managed Database

1. Create a PostgreSQL database in DigitalOcean
2. Enable TimescaleDB extension in the database settings
3. Get connection details from the dashboard
4. Update `.env` file with connection details

### Option 4: Self-Hosted VPS

If you have a VPS (DigitalOcean, Linode, AWS EC2, etc.):

1. Follow the [Local Setup](#local-setup) instructions on your VPS
2. Configure firewall to allow PostgreSQL connections (port 5432)
3. Update `postgresql.conf` to listen on all interfaces:
   ```
   listen_addresses = '*'
   ```
4. Update `pg_hba.conf` to allow remote connections:
   ```
   host    all    all    0.0.0.0/0    md5
   ```
5. Restart PostgreSQL service
6. Update `.env` with your VPS IP address

---

## 📊 Database Schema

### Tables

#### 1. `sensor_readings` (Hypertable)
Time-series data for all sensor readings:
- `time` - Timestamp
- `device_id` - Device identifier
- `battery_voltage`, `battery_current`, `battery_power`
- `main_voltage`, `main_current`, `main_power`
- `light_intensity`
- `led_status`, `led2_status`, `led4_status`, `emergency_light_status`
- `power_cut_status`

#### 2. `power_cut_events` (Hypertable)
Historical power cut events:
- `id` - Primary key
- `device_id` - Device identifier
- `start_time`, `end_time` - Event timestamps
- `duration_ms` - Duration in milliseconds
- `start_voltage`, `end_voltage`, `voltage_drop`
- `energy_consumed_mwh` - Energy consumed during outage

#### 3. `command_logs` (Hypertable)
Command execution logs:
- `id` - Primary key
- `device_id` - Device identifier
- `time` - Timestamp
- `command_type` - Type of command
- `command_value` - Command value
- `topic` - MQTT topic
- `message` - Full message

### Continuous Aggregates

#### `sensor_readings_hourly`
Pre-computed hourly statistics for faster queries:
- Hourly averages of all sensor values
- Automatically refreshed every hour
- Reduces query time for historical data

### Data Retention (Optional)

Uncomment retention policies in `schema.sql` to automatically delete old data:
- Sensor readings: 90 days
- Power cut events: 365 days
- Command logs: 90 days

---

## 🔍 Verification

### Check Database Connection

```bash
# Test connection
psql -h localhost -U postgres -d iot_monitoring -c "SELECT NOW();"
```

### Check TimescaleDB Extension

```sql
-- Connect to database
psql -h localhost -U postgres -d iot_monitoring

-- Check if TimescaleDB is installed
SELECT * FROM pg_extension WHERE extname = 'timescaledb';

-- Check hypertables
SELECT * FROM timescaledb_information.hypertables;
```

### Check Data Insertion

After starting the server, verify data is being inserted:

```sql
-- Check sensor readings
SELECT COUNT(*) FROM sensor_readings;

-- Check latest readings
SELECT * FROM sensor_readings ORDER BY time DESC LIMIT 10;
```

---

## 🐛 Troubleshooting

### Connection Refused

**Error**: `Connection refused` or `ECONNREFUSED`

**Solutions**:
1. Check if PostgreSQL is running:
   ```bash
   sudo systemctl status postgresql  # Linux
   brew services list  # macOS
   ```
2. Check PostgreSQL is listening on correct port:
   ```bash
   sudo netstat -tlnp | grep 5432
   ```
3. Verify firewall settings
4. Check `postgresql.conf` for `listen_addresses`

### Authentication Failed

**Error**: `password authentication failed`

**Solutions**:
1. Verify password in `.env` file
2. Check `pg_hba.conf` authentication method
3. Reset PostgreSQL password:
   ```sql
   ALTER USER postgres WITH PASSWORD 'new_password';
   ```

### TimescaleDB Extension Not Found

**Error**: `extension "timescaledb" does not exist`

**Solutions**:
1. Install TimescaleDB extension (see [Local Setup](#local-setup))
2. Create extension manually:
   ```sql
   CREATE EXTENSION IF NOT EXISTS timescaledb;
   ```

### Hypertable Creation Failed

**Error**: `table is not a hypertable`

**Solutions**:
1. Drop existing table and recreate:
   ```sql
   DROP TABLE IF EXISTS sensor_readings CASCADE;
   -- Then run schema.sql again
   ```
2. Check if TimescaleDB extension is enabled

### Port Already in Use

**Error**: `Port 3000 is already in use`

**Solutions**:
1. Change PORT in `.env` file
2. Kill process using port 3000:
   ```bash
   lsof -ti:3000 | xargs kill  # macOS/Linux
   netstat -ano | findstr :3000  # Windows (then kill PID)
   ```

### Environment Variables Not Loading

**Error**: Using default values instead of `.env` values

**Solutions**:
1. Install dotenv: `npm install dotenv`
2. Verify `.env` file exists in `web/` directory
3. Check `.env` file syntax (no spaces around `=`)
4. Restart the server

---

## 📈 Performance Optimization

### Chunk Interval

For high-frequency data, adjust chunk interval:

```sql
SELECT set_chunk_time_interval('sensor_readings', INTERVAL '1 day');
```

### Compression

Enable compression for older data:

```sql
ALTER TABLE sensor_readings SET (
  timescaledb.compress,
  timescaledb.compress_segmentby = 'device_id'
);

SELECT add_compression_policy('sensor_readings', INTERVAL '7 days');
```

### Indexes

Additional indexes for specific queries:

```sql
CREATE INDEX idx_sensor_readings_power_cut 
ON sensor_readings(time DESC) 
WHERE power_cut_status = 'POWER_CUT';
```

---

## 🔐 Security Best Practices

1. **Use Strong Passwords**: Never use default passwords in production
2. **Limit Network Access**: Use firewall rules to restrict database access
3. **Use SSL/TLS**: Enable SSL connections for cloud databases
4. **Regular Backups**: Set up automated backups
5. **Environment Variables**: Never commit `.env` file to version control
6. **Least Privilege**: Create dedicated database user with minimal permissions

---

## 📚 Additional Resources

- [TimescaleDB Documentation](https://docs.timescale.com/)
- [PostgreSQL Documentation](https://www.postgresql.org/docs/)
- [TimescaleDB Tutorials](https://docs.timescale.com/tutorials/)
- [Hypertables Guide](https://docs.timescale.com/use-timescale/latest/hypertables/)

---

## ✅ Quick Start Checklist

- [ ] PostgreSQL installed
- [ ] TimescaleDB extension installed
- [ ] Database created
- [ ] Schema initialized
- [ ] `.env` file configured
- [ ] Node.js dependencies installed (including `dotenv`)
- [ ] Server starts without errors
- [ ] Database connection verified
- [ ] Data insertion verified

---

**Need Help?** Check the troubleshooting section or refer to the main project documentation.

