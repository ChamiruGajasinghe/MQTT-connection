#!/bin/bash

# TimescaleDB Database Initialization Script
# This script creates the database and sets up the schema

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}TimescaleDB Initialization Script${NC}"
echo -e "${GREEN}========================================${NC}"

# Load environment variables
if [ -f .env ]; then
    export $(cat .env | grep -v '^#' | xargs)
    echo -e "${YELLOW}✓ Loaded environment variables from .env${NC}"
else
    echo -e "${YELLOW}⚠ No .env file found. Using defaults.${NC}"
fi

# Database connection parameters
DB_HOST="${TIMESCALEDB_HOST:-localhost}"
DB_PORT="${TIMESCALEDB_PORT:-5432}"
DB_NAME="${TIMESCALEDB_DATABASE:-iot_monitoring}"
DB_USER="${TIMESCALEDB_USER:-postgres}"
DB_PASSWORD="${TIMESCALEDB_PASSWORD:-postgres}"

echo ""
echo -e "${YELLOW}Database Configuration:${NC}"
echo "  Host: $DB_HOST"
echo "  Port: $DB_PORT"
echo "  Database: $DB_NAME"
echo "  User: $DB_USER"
echo ""

# Export password for psql
export PGPASSWORD="$DB_PASSWORD"

# Check if database exists
echo -e "${YELLOW}Checking if database exists...${NC}"
DB_EXISTS=$(psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -lqt | cut -d \| -f 1 | grep -w "$DB_NAME" | wc -l)

if [ "$DB_EXISTS" -eq 0 ]; then
    echo -e "${YELLOW}Creating database: $DB_NAME${NC}"
    psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -c "CREATE DATABASE $DB_NAME;"
    echo -e "${GREEN}✓ Database created${NC}"
else
    echo -e "${GREEN}✓ Database already exists${NC}"
fi

# Run schema script
echo ""
echo -e "${YELLOW}Running schema script...${NC}"
SCHEMA_FILE="$(dirname "$0")/schema.sql"

if [ ! -f "$SCHEMA_FILE" ]; then
    echo -e "${RED}✗ Schema file not found: $SCHEMA_FILE${NC}"
    exit 1
fi

psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" -f "$SCHEMA_FILE"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Schema initialized successfully${NC}"
else
    echo -e "${RED}✗ Schema initialization failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Database initialization complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "You can now start the web server with:"
echo "  npm start"
echo ""

