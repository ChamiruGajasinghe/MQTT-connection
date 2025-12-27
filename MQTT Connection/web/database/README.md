# Database Directory

This directory contains all database-related files for the TimescaleDB setup.

## Files

- **`schema.sql`** - Database schema with TimescaleDB hypertables
- **`init-db.sh`** - Database initialization script for Linux/macOS
- **`init-db.ps1`** - Database initialization script for Windows PowerShell
- **`README.md`** - This file

## Quick Start

### 1. Ensure PostgreSQL and TimescaleDB are installed

See [TIMESCALEDB_SETUP.md](../../TIMESCALEDB_SETUP.md) for installation instructions.

### 2. Run initialization script

**Linux/macOS:**
```bash
chmod +x init-db.sh
./init-db.sh
```

**Windows (PowerShell):**
```powershell
.\init-db.ps1
```

### 3. Verify installation

```sql
-- Connect to database
psql -h localhost -U postgres -d iot_monitoring

-- Check hypertables
SELECT * FROM timescaledb_information.hypertables;
```

## Schema Overview

The schema creates three main hypertables:

1. **sensor_readings** - Time-series sensor data
2. **power_cut_events** - Power cut event history
3. **command_logs** - Command execution logs

All tables are automatically partitioned by time for optimal performance.

## Manual Setup

If the scripts don't work, you can run the schema manually:

```bash
psql -h localhost -U postgres -d iot_monitoring -f schema.sql
```

## Troubleshooting

See the main [TIMESCALEDB_SETUP.md](../../TIMESCALEDB_SETUP.md) documentation for troubleshooting tips.

