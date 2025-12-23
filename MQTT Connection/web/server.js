// MQTT IoT Monitoring Web Server - Global Access
const express = require('express');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;

// Middleware
app.use(express.static(__dirname));
app.use(express.json());

// Serve main dashboard
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'dashboard.html'));
});

// Serve dashboard explicitly
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
app.get('/api/health', (req, res) => {
    res.json({ 
        status: 'online',
        timestamp: new Date().toISOString(),
        uptime: process.uptime()
    });
});

// Start server on all network interfaces (0.0.0.0) for global access
app.listen(PORT, '0.0.0.0', () => {
    console.log('=============================================');
    console.log('🌐 IoT Monitoring System - GLOBAL ACCESS');
    console.log('=============================================');
    console.log(`📡 Local Access:    http://localhost:${PORT}`);
    console.log(`🌍 Network Access:  http://[YOUR_IP]:${PORT}`);
    console.log(`📊 Dashboard:       http://localhost:${PORT}/dashboard.html`);
    console.log(`📈 History:         http://localhost:${PORT}/history.html`);
    console.log(`🎛️  Control:         http://localhost:${PORT}/global-control.html`);
    console.log(`🔌 MQTT Broker:     wss://broker.hivemq.com:8884/mqtt`);
    console.log('=============================================');
    console.log('💡 For internet access:');
    console.log('   1. Get your public IP from: https://whatismyipaddress.com');
    console.log('   2. Forward port', PORT, 'in your router settings');
    console.log('   3. Access via: http://[PUBLIC_IP]:' + PORT);
    console.log('   OR use a service like: ngrok, localtunnel, or deploy to cloud');
    console.log('=============================================');
});
