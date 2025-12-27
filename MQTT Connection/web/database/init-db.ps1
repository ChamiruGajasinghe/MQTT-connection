# TimescaleDB Database Initialization Script (PowerShell)
# This script creates the database and sets up the schema for Windows

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Green
Write-Host "TimescaleDB Initialization Script" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

# Load environment variables from .env file
if (Test-Path ".env") {
    Get-Content ".env" | ForEach-Object {
        if ($_ -match '^\s*([^#][^=]+)=(.*)$') {
            $name = $matches[1].Trim()
            $value = $matches[2].Trim()
            [Environment]::SetEnvironmentVariable($name, $value, "Process")
        }
    }
    Write-Host "✓ Loaded environment variables from .env" -ForegroundColor Yellow
} else {
    Write-Host "⚠ No .env file found. Using defaults." -ForegroundColor Yellow
}

# Database connection parameters
$DB_HOST = if ($env:TIMESCALEDB_HOST) { $env:TIMESCALEDB_HOST } else { "localhost" }
$DB_PORT = if ($env:TIMESCALEDB_PORT) { $env:TIMESCALEDB_PORT } else { "5432" }
$DB_NAME = if ($env:TIMESCALEDB_DATABASE) { $env:TIMESCALEDB_DATABASE } else { "iot_monitoring" }
$DB_USER = if ($env:TIMESCALEDB_USER) { $env:TIMESCALEDB_USER } else { "postgres" }
$DB_PASSWORD = if ($env:TIMESCALEDB_PASSWORD) { $env:TIMESCALEDB_PASSWORD } else { "postgres" }

Write-Host ""
Write-Host "Database Configuration:" -ForegroundColor Yellow
Write-Host "  Host: $DB_HOST"
Write-Host "  Port: $DB_PORT"
Write-Host "  Database: $DB_NAME"
Write-Host "  User: $DB_USER"
Write-Host ""

# Set password environment variable for psql
$env:PGPASSWORD = $DB_PASSWORD

# Check if database exists
Write-Host "Checking if database exists..." -ForegroundColor Yellow
$dbList = & psql -h $DB_HOST -p $DB_PORT -U $DB_USER -lqt 2>&1
$dbExists = $dbList | Select-String -Pattern "\b$DB_NAME\b"

if (-not $dbExists) {
    Write-Host "Creating database: $DB_NAME" -ForegroundColor Yellow
    & psql -h $DB_HOST -p $DB_PORT -U $DB_USER -c "CREATE DATABASE $DB_NAME;" 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Database created" -ForegroundColor Green
    } else {
        Write-Host "✗ Failed to create database" -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "✓ Database already exists" -ForegroundColor Green
}

# Run schema script
Write-Host ""
Write-Host "Running schema script..." -ForegroundColor Yellow
$SCHEMA_FILE = Join-Path $PSScriptRoot "schema.sql"

if (-not (Test-Path $SCHEMA_FILE)) {
    Write-Host "✗ Schema file not found: $SCHEMA_FILE" -ForegroundColor Red
    exit 1
}

& psql -h $DB_HOST -p $DB_PORT -U $DB_USER -d $DB_NAME -f $SCHEMA_FILE

if ($LASTEXITCODE -eq 0) {
    Write-Host "✓ Schema initialized successfully" -ForegroundColor Green
} else {
    Write-Host "✗ Schema initialization failed" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "✓ Database initialization complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "You can now start the web server with:"
Write-Host "  npm start"
Write-Host ""

