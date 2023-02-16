Add this to your TreeNodesModel:


    <Action ID="ImageToBlackboard">
      <metadata subcategory="Perception"/>
      <description>
        <p>
                    Put the contents of an image topic on the blackboard
                </p>
      </description>
      <input_port name="input_topic" default="/scene_camera/color/image_raw">The topic this behaviour subscribes to.</input_port>
      <output_port name="output_port" default="{blackboard_image}">The port the image will be placed.</output_port>
    </Action>
    <Action ID="ImageToFile">
      <metadata subcategory="Perception"/>
      <description>
        <p>
                    Save the contents of an image topic to a file
                </p>
      </description>
      <input_port name="input_topic" default="/scene_camera/color/image_raw">The topic this behaviour subscribes to.</input_port>
      <output_port name="output_file" default="/opt/moveit_studio/user_ws/image_file.jpg">The full path and filename to save the image.</output_port>
    </Action>
    <Action ID="BlackboardImageToFile">
      <metadata subcategory="Perception"/>
      <description>
        <p>
                    Save the contents of a blackboard image to a file
                </p>
      </description>
      <input_port name="input_image" default="{blackboard_image}">The blackboard port where the image can be found.</input_port>
      <output_port name="output_file" default="/opt/moveit_studio/user_ws/image_file.jpg">The full path and filename to save the image.</output_port>
    </Action>


Add this to your site_config.yaml:

  behavior_loader_plugins:
    perception_services:
      - "perception_services::PerceptionServicesBehaviorsLoader"  # Your new custom Behavior loader plugin
