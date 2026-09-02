#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace liorf::pcm {

// PCM is recomputed as registrations arrive, so membership in its current
// maximum clique is provisional. Graph factors, however, cannot presently be
// retracted. This policy turns repeated PCM membership into a monotonic set of
// committed candidates.
struct CommitmentConfig {
  std::size_t min_clique_size = 5;
  std::size_t min_consecutive_acceptances = 3;
};

inline std::string validate(const CommitmentConfig& config) {
  if (config.min_clique_size < 2)
    return "min_clique_size must be at least 2";
  if (config.min_consecutive_acceptances < 1)
    return "min_consecutive_acceptances must be at least 1";
  return {};
}

class CommitmentTracker {
 public:
  explicit CommitmentTracker(CommitmentConfig config = {})
      : config_(config) {}

  // Candidate indices are stable because map fusion only appends to its loop
  // queue. Invalid or duplicate clique indices are ignored defensively.
  // Once returned, an index remains committed: this mirrors the current
  // factor graph, whose published factors are also monotonic.
  const std::vector<int>& update(
      std::size_t candidate_count,
      const std::vector<int>& maximum_clique) {
    if (candidate_count < acceptance_streak_.size())
      reset();
    acceptance_streak_.resize(candidate_count, 0);
    committed_.resize(candidate_count, false);

    std::vector<bool> accepted(candidate_count, false);
    std::size_t accepted_count = 0;
    for (const int index : maximum_clique) {
      if (index < 0 || static_cast<std::size_t>(index) >= candidate_count ||
          accepted[static_cast<std::size_t>(index)])
        continue;
      accepted[static_cast<std::size_t>(index)] = true;
      ++accepted_count;
    }

    const bool clique_supported =
      accepted_count >= config_.min_clique_size;
    for (std::size_t index = 0; index < candidate_count; ++index) {
      // Tenure accumulated in a small clique is not evidence for the
      // publication policy. Start (or continue) the streak only after the
      // whole clique has reached the configured support floor.
      if (!clique_supported || !accepted[index]) {
        acceptance_streak_[index] = 0;
        continue;
      }
      if (acceptance_streak_[index] <
          std::numeric_limits<std::size_t>::max())
        ++acceptance_streak_[index];
    }

    if (clique_supported) {
      for (std::size_t index = 0; index < candidate_count; ++index) {
        if (accepted[index] &&
            acceptance_streak_[index] >=
                config_.min_consecutive_acceptances)
          committed_[index] = true;
      }
    }

    committed_indices_.clear();
    for (std::size_t index = 0; index < committed_.size(); ++index) {
      if (committed_[index])
        committed_indices_.push_back(static_cast<int>(index));
    }
    return committed_indices_;
  }

  const std::vector<int>& committed() const noexcept {
    return committed_indices_;
  }

  void reset() {
    acceptance_streak_.clear();
    committed_.clear();
    committed_indices_.clear();
  }

 private:
  CommitmentConfig config_;
  std::vector<std::size_t> acceptance_streak_;
  std::vector<bool> committed_;
  std::vector<int> committed_indices_;
};

}  // namespace liorf::pcm
