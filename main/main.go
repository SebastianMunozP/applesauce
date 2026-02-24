package main

import (
	"context"
	"flag"
	"os"
	"os/signal"
	"syscall"

	"github.com/biotinker/applesauce"
	"github.com/biotinker/applesauce/internal/creds"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

func main() {
	credsPath := flag.String("creds", "", "path to robot credentials JSON file")
	flag.Parse()

	logger := logging.NewDebugLogger("applesauce")

	if *credsPath == "" {
		logger.Fatal("-creds flag is required")
	}
	robotCreds, err := creds.Load(*credsPath)
	if err != nil {
		logger.Fatal(err)
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	machine, err := client.New(
		ctx,
		robotCreds.Address,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			robotCreds.EntityID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: robotCreds.APIKey,
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}
	defer machine.Close(context.Background())

	logger.Info("Connected to robot")
	logger.Info("Resources:", machine.ResourceNames())

	r, err := applesauce.NewRobot(ctx, machine, logger)
	if err != nil {
		logger.Fatal(err)
	}
	_ = r

	if err := applesauce.Run(ctx, r); err != nil {
		logger.Fatal(err)
	}
}
