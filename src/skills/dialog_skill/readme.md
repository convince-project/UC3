# Dialog Skill

The dialog pipeline is supposed to define the sequences of actions of a generic conversation between the robot and the user. In the context of Palazzo Madama, the robot is assumed to have conversations with visitors as a tour guide. For this reason, the dialog is settled as a question-answer dialog, where visitors are supposed to ask information about anything related the museum, the tour and the robot itself, while the robot is supposed to provide answers and wait for other questions or deciding when continuing the tour.

# State Machine
<!-- ![Dialog Flow Chart](images/Dialog_pipeline.drawio.svg) -->
<figure>
  <p align="center">
    <img src="images/Dialog_pipeline.drawio.svg" alt="Alt Text" width="75%">
  </p>
  <figcaption align="center">
    <em>Dialog Flow Chart (made with 
    <a href="https://app.diagrams.net/" target="_blank">diagrams.net</a>, 
    last update: May 14th 2025)</em>
  </figcaption>
</figure>



The dialog skill behavior follows the flow chart presented above:
- The starting state is `IDLE`
- If a tick is received, then the `WaitForInteract` state is supposed to wait for the interaction of the user and to pass it to the next state. If nothing is received, wait. When an interaction is received, go to the next state.
- The `ManageContext` state receives the interaction and based on the context. There are some predefined replies, it delivers the request to a specific LLM, or directly replies with predefined actions. If the context requires a specific LLM, then go to the `CheckDuplicate` state; if it needs a predefined reply, speak and at the end of the speech come back to the state `WaitForInteraction`; if it is an invite to start/end/continue the tour, get back to the `IDLE` state.
- The `CheckDuplicate` state is supposed to recognize if the semantical meaning of the received question sentence has already been heard within the dialog.
- If it's the first time that the question has been asked, then go to the `AnswerAndSpeak` state. This state provides LLMs that generate replies based on the context of the interaction. Until now, there are 2 LLMs: one that is specialized for questions related to Palazzo Madama and to the CONVINCE project, and another one that is specialized to manage questions related to R1 itself, as well as general questions. This design choice has been made to avoid allucinations and long execution times due to the length of the prompt that needs to be processed to include all the relevant information. After generating the answer, speak and at the end of the speech, come back to the state `WaitForInteraction`,
- If a similar question has already been asked within the dialog session, the `ShortenAndSpeak` state is supposed to provide a resumed version of the previous answer, rather than replying the same information. After generating the answer, speak and at the end of the speech, come back to the state `WaitForInteraction`,

# Components
The dialog skill represents a ROS2 node acting as a ROS2 Service Client, which interacts with the following components, implementing ROS2 Service Servers:
- **Dialog Component**: The dialog component provides the majoirity of the services required by the dialog skills, except for the speech synthesis, the speech recognition and the language settings. The dialog component is responsible to manage the dialog session, to provide the answer to the questions and to manage the dialog state. The dialog component is also responsible to manage the LLM, which is used to generate the answers.
- **Scheduler Component**: The scheduler component is responsible to provide the language of the conversation, based on the tour settings.

The Dialog Component also interacts with the following components:
- **Text-To-Speech Component**: The text to speech component is responsible to provide the speech synthesis service, which is used to generate the speech from the text.
- **Scheduler Component**: The scheduler component is responsible to provide the POI of the tour, and to change the PoI in case the user asks to continue the tour.
- **Speech-To-Text Component**: It is used to translate the voice interactions into text.

Below is a graphical representation of the components and their interactions. Here, grey boxes represent ROS2 nodes, orange boxes represent ROS2 services and blue boxes represent ROS2 topics. If the ROS2 service is on the left side of the box, it means that the service is provided by the node (i.e., the node implements them as ROS2 services server), otherwise it means that the service is required by the node (i.e., the node implements them as ROS2 service clients).

<figure style="text-align: center;">
    <img src="images/dialog_network_viz.png" alt="Alt Text" style="width:100%; height:auto;">
    <figcaption style="margin-top: 0.5em; font-style: italic; color: #555;">
        ROS2 services of Dialog Component and Skill (made automatically with 
        <a href="https://github.com/ros2/ros_network_viz">ros network viz</a>
        , last update: April 29th, 2025)
    </figcaption>
</figure>

# Component-Skill-State Machine Communication
Below is a graphical representation of the communication between the component, the skill and the state machine. The arrows represent the communication between the entities. When a state of the state machine presents multiple input-output arrows, the green arrows represent the sequential input-output flow of the state machine's communication with the skill.

<figure style="text-align: center;">
    <img src="images/SequenceDiagram.drawio.svg" alt="Alt Text" style="width:75%; height:auto;">
    <figcaption style="margin-top: 0.5em; font-style: italic; color: #555;">
        Sequence Diagram representing the communication among the Dialog Component, Skill and State Machine (made with 
        <a href="https://app.diagrams.net/" target="_blank" style="color: #007acc; text-decoration: none;">diagrams.net</a>, 
        last update: May 14th 2025)
    </figcaption>
</figure>