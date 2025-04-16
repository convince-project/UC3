import rclpy
from rclpy.node import Node
from dialog_interfaces.srv import RememberInteractions
import time
from sentence_transformers import SentenceTransformer
import numpy as np

class InteractionService(Node):

    def __init__(self):
        super().__init__('interaction_service')
        self.srv = self.create_service(RememberInteractions, 'remember_interactions', self.get_previous_similar_interaction)

        self.saved_interactions = []
        self.saved_embeddings = None

        start = time.time()
        self.model = SentenceTransformer("sentence-transformers/distiluse-base-multilingual-cased-v1")
        # self.model = SentenceTransformer("all-MiniLM-L6-v2")
        print("Model loaded in %s seconds" % (time.time() - start))

        self.get_logger().info('Ready to compare interactions!')

    # return the index of a previous interaction that is similar to the current one if found, otherwise -1
    def get_previous_similar_interaction(self, request, response):

        # print("Previous interactions: ", self.saved_interactions)
        print("Current interaction: ", request.interaction)

        # start = time.time()
        embedding = self.model.encode([request.interaction])
        # print("Sentence encoded in %s seconds" % (time.time() - start))
        # print(embedding.shape)

        # start = time.time()
        similarities = self.model.similarity(embedding, self.saved_embeddings) if self.saved_embeddings is not None else np.array([[0]])
        # print("Similarities calculated in %s seconds" % (time.time() - start))
        # print(similarities)
        # print(similarities.shape)

        max_sim_index = np.unravel_index(np.argmax(similarities, axis=None), similarities.shape)
        max_sim_value = similarities[max_sim_index].item()
        # print(max_sim_value, max_sim_index)

        if max_sim_value > 0.5:
            # print(self.saved_interactions)
            # print(similarities)
            # TODO: Discuss if skipping the embedding is a good idea, or if it's better computing the average of the duplicate embeddings
            duplicate_index = int(max_sim_index[1])
        else:
            # start = time.time()
            self.saved_interactions.append(request.interaction)
            self.saved_embeddings = np.concatenate(
                (self.saved_embeddings, embedding)
            ) if self.saved_embeddings is not None else embedding
            # print("Embeddings concatenated in %s seconds" % (time.time() - start))
            # print(self.saved_embeddings.shape)

            duplicate_index = -1
        
        # self.response.data = f"{self.saved_interactions[self.duplicate_index]}"
        response.index = duplicate_index
        if duplicate_index != -1:
            self.get_logger().info(f'Duplicate interaction detected: previous "{self.saved_interactions[duplicate_index]}" and current "{request.interaction}"')

        print("-" * 50)

        return response


def main(args=None):
    rclpy.init(args=args)

    interaction_service = InteractionService()

    rclpy.spin(interaction_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()