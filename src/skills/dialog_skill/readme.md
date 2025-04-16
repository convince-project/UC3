# Dialog Skill

The dialog pipeline is supposed to define the sequences of actions of a generic conversation between the robot and the user. In the context of Palazzo Madama, the robot is assumed to have conversations with visitors as a tour guide. For this reason, the dialog is settled as a question-answer dialog, where visitors are supposed to ask information about anything related the museum, the tour and the robot itself, while the robot is supposed to provide answers and wait for other questions or deciding when continuing the tour.

<!-- ![Dialog Flow Chart](images/Dialog_pipeline.drawio.svg) -->
<p align="center">
    <img src="images/Dialog_pipeline.drawio.svg" alt="Alt Text" style="width:50%; height:auto;">
</p>


The dialog skill behavior follows the flow chart presented above:
- The starting state is `IDLE`
- If a tick is received, then the `WaitForInteract` state is supposed to wait for the text and to pass it to the next state. If not, wait. When a text interaction is received, go to the next state.
- The `SetLanguage` state takes the interaction and sets the language based on the question (Does it make sense? Couldn't we use the LLM just reply in the language of the question?). Then go to the next state
- The `CheckDuplicate` state is supposed to recognize if the semantical meaning of the received question sentence has already been heard within the dialog.
- If it's the first time that the question has been asked, then go to the `Interpret` state, which is supposed to understand if the sentence is another question or the invite to continue the tour. If the sentence is a question, then go to the `AnswerAndSpeak` state, otherwise it means that one among the invitation to continue, starting or ending the tour has been received, therefore come back to the `IDLE` state.
- If a similar question has already been asked within the dialog session, the `ShortenAndSpeak` is supposed to provide a resumed version of the previous answer, rather than replying the same information. After generating the answer, speak and at the end of the speech, come back to the state `WaitForInteraction`
- The `AnswerAndSpeak` state is supposed to generate an answer and to estate it verbally. Analogously to the `ShortenAndSpeak` state, at the end of the speech, come back to the `WaitForInteraction` state