# Kinect Multi-Feed Viewer

Quick WPF app to view all Kinect camera feeds with toggleable modes.

## Prerequisites

1. **Kinect for Windows SDK 2.0**
   - Download: https://www.microsoft.com/en-us/download/details.aspx?id=44561
   - Install before building this project

2. **Kinect v2 sensor** (the Xbox One version works too with adapter)

3. **.NET 6.0 SDK** or later

## Features

- **Color Camera** - Full RGB camera feed (1920x1080)
- **Depth Camera** - Depth map visualization (512x424)
- **Infrared Camera** - IR feed (512x424)
- **Body Tracking** - Skeleton overlay with 25 joint tracking

### Stream Behavior

- Color and Infrared are mutually exclusive (hardware limitation) - enabling one disables the other
- Depth and Body Tracking can run alongside any other stream
- All feeds overlay when enabled together (depth/body over color, etc)

## Building

```bash
cd KinectViewer
dotnet build
dotnet run
```

Or open `KinectViewer.csproj` in Visual Studio 2022 and hit F5.

## Usage

1. Connect your Kinect v2 sensor
2. Launch the app
3. Check the boxes at the bottom to enable feeds:
   - **Color** - RGB camera
   - **Depth** - Grayscale depth map (darker = closer)
   - **Infrared** - IR illuminator view
   - **Body Tracking** - Green skeleton overlay

The status bar shows connection state.

## Troubleshooting

**"No Kinect detected!"**
- Make sure Kinect is plugged into USB 3.0 port
- Check Device Manager for Kinect devices
- Reinstall Kinect SDK drivers

**Build errors about Microsoft.Kinect.dll**
- Update the `HintPath` in `KinectViewer.csproj` if SDK is installed elsewhere
- Default path: `C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Assemblies\`

**Feeds not showing**
- Kinect takes a few seconds to initialize after plugging in
- Try unchecking/rechecking the feed
- Restart the app

## Notes

- Depth and IR are lower res (512x424) vs Color (1920x1080)
- Body tracking uses the depth sensor, so it works with depth or IR enabled
- Gray16 pixel format for depth/IR gives you raw sensor values
- Skeleton coordinates are mapped to color space for proper overlay alignment
