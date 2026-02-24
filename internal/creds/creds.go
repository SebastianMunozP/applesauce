package creds

import (
	"encoding/json"
	"fmt"
	"os"
)

// RobotCredentials holds the connection details for a Viam robot.
type RobotCredentials struct {
	Address  string `json:"address"`
	EntityID string `json:"entity_id"`
	APIKey   string `json:"api_key"`
}

// Load reads and parses robot credentials from a JSON file.
func Load(path string) (*RobotCredentials, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("reading credentials file: %w", err)
	}
	var c RobotCredentials
	if err := json.Unmarshal(data, &c); err != nil {
		return nil, fmt.Errorf("parsing credentials file: %w", err)
	}
	return &c, nil
}
