import limelight
import limelightresults
import json
import time

discovered_limelights = limelight.discover_limelights(debug=True)
print("discovered limelights:", discovered_limelights)

if discovered_limelights:
    limelight_address = discovered_limelights[1]
    ll = limelight.Limelight(limelight_address)
    results = ll.get_results()
    status = ll.get_status()
    print("-----")
    print("targeting results:", results)
    print("-----")
    print("status:", status)
    print("-----")
    print("temp:", ll.get_temp())
    print("-----")
    print("name:", ll.get_name())
    print("-----")
    print("fps:", ll.get_fps())
    print("-----")
    print("hwreport:", ll.hw_report())

    ll.enable_websocket()

    # print the current pipeline settings
    print(ll.get_pipeline_atindex(0))

    # update the current pipeline and flush to disk
    pipeline_update = {"area_max": 98.7, "area_min": 1.98778}
    ll.update_pipeline(json.dumps(pipeline_update), flush=1)

    print(ll.get_pipeline_atindex(0))

    # switch to pipeline 1
    ll.pipeline_switch(1)

    # update custom user data
    ll.update_python_inputs([4.2, 0.1, 9.87])

    try:
        while True:
            result = ll.get_latest_results()
            parsed_result = limelightresults.parse_results(result)
            if parsed_result is not None:
                print(
                    "valid targets: ",
                    parsed_result.validity,
                    ", pipelineIndex: ",
                    parsed_result.pipeline_id,
                    ", Targeting Latency: ",
                    parsed_result.targeting_latency,
                    ", Fiducial Results: ",
                    parsed_result.fiducialResults,
                )
                # for tag in parsed_result.fiducialResults:
                #    print(tag.robot_pose_target_space, tag.fiducial_id)
            time.sleep(1)  # Set this to 0 for max fps

    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
    finally:
        ll.disable_websocket()
