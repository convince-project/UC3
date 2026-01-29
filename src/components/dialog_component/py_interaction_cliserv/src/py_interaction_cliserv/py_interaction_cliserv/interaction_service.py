import rclpy
from rclpy.node import Node
from dialog_interfaces.srv import RememberInteractions, ResetState
import time
from sentence_transformers import SentenceTransformer
import numpy as np

class InteractionService(Node):

    def __init__(self):
        super().__init__('py_dialog_component')
        self.srv = self.create_service(RememberInteractions, '/DialogComponent/RememberInteractions', self.get_previous_similar_interaction)
        self.reset_srv = self.create_service(ResetState, "/DialogComponent/PyResetState", self.reset_state)

        self.saved_interactions = dict()
        self.saved_embeddings = dict()

        start = time.time()

        # dict of sentence models and their thresholds
        sentence_model_thresholds = {
            "sentence-transformers/all-MiniLM-L6-v2": 0.5,
            "sentence-transformers/distiluse-base-multilingual-cased-v1": 1, # needs to be tuned
            "intfloat/multilingual-e5-large-instruct": 0.925,
            "paraphrase-multilingual-MiniLM-L12-v2": 0.8
        }

        # TODO: make it a parameter
        model_name = "paraphrase-multilingual-MiniLM-L12-v2"

        self.model = SentenceTransformer(model_name)

        self.threshold = sentence_model_thresholds[model_name]
        self.get_logger().info(f"Model {model_name} loaded with threshold {self.threshold}")
        self.get_logger().info(f"Model loaded in {time.time() - start} seconds")

        self.get_logger().info('Ready to compare interactions!')

    # return the index of a previous interaction that is similar to the current one if found, otherwise -1
    def get_previous_similar_interaction(self, request, response):

        if request.is_beginning_of_conversation:
            self.saved_interactions = dict()
            self.saved_embeddings = dict()
            self.get_logger().info("New conversation started, resetting saved interactions and embeddings")

        if request.context not in self.saved_interactions and request.context not in self.saved_embeddings:
            self.saved_interactions[request.context] = []
            self.saved_embeddings[request.context] = None

        # self.get_logger().info("Previous interactions: ", self.saved_interactions)
        self.get_logger().info(f"Current interaction: {request.interaction}")

        # start = time.time()
        embedding = self.model.encode([request.interaction])
        # self.get_logger().info("Sentence encoded in %s seconds" % (time.time() - start))
        # self.get_logger().info(embedding.shape)

        # start = time.time()
        similarities = self.model.similarity(embedding, self.saved_embeddings[request.context]) if self.saved_embeddings[request.context] is not None else np.array([[0]])
        # self.get_logger().info("Similarities calculated in %s seconds" % (time.time() - start))
        # self.get_logger().info(similarities)
        # self.get_logger().info(similarities.shape)

        max_sim_index = np.unravel_index(np.argmax(similarities, axis=None), similarities.shape)
        max_sim_value = similarities[max_sim_index].item()
        # self.get_logger().info(max_sim_value, max_sim_index)

        if max_sim_value > self.threshold:
            # self.get_logger().info(self.saved_interactions)
            # self.get_logger().info(similarities)
            # TODO: Discuss if skipping the embedding is a good idea, or if it's better computing the average of the duplicate embeddings
            duplicate_index = int(max_sim_index[1])
        else:
            # start = time.time()
            self.saved_interactions[request.context].append(request.interaction)
            self.saved_embeddings[request.context] = np.concatenate(
                (self.saved_embeddings[request.context], embedding)
            ) if self.saved_embeddings[request.context] is not None else embedding
            # self.get_logger().info("Embeddings concatenated in %s seconds" % (time.time() - start))
            # self.get_logger().info(self.saved_embeddings.shape)

            duplicate_index = -1
        
        # self.response.data = f"{self.saved_interactions[self.duplicate_index]}"
        response.index = duplicate_index
        response.is_ok = True
        if duplicate_index != -1:
            self.get_logger().info(f'Duplicate interaction detected: previous "{self.saved_interactions[request.context][duplicate_index]}" and current "{request.interaction}"')

        self.get_logger().info("-" * 50)

        return response

        # return the index of a previous interaction that is similar to the current one if found, otherwise -1
    def reset_state(self, request, response):

        self.saved_interactions = dict()
        self.saved_embeddings = dict()
        self.get_logger().info("New conversation started, resetting saved interactions and embeddings")

        response.is_ok = True

        return response


def main(args=None):
    rclpy.init(args=args)

    interaction_service = InteractionService()

    rclpy.spin(interaction_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()