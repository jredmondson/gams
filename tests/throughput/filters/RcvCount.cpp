 
#include "RcvCount.h"
#include <time.h>

typedef madara::knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer  Integer;

filters::RcvCount::RcvCount(
  utility::MovingAverage & latencies, bool no_apply)
  : initialized_(false), no_apply_(no_apply), latencies_(latencies)
{
}

filters::RcvCount::~RcvCount()
{
}

void
filters::RcvCount::filter(
  madara::knowledge::KnowledgeMap & records,
  const madara::transport::TransportContext & transport_context,
  madara::knowledge::Variables & vars)
{
  const std::string originator = transport_context.get_originator();

  if (!initialized_)
  {
    count_.set_name(".receives", vars);
    filtered_messages_.set_name(".deleted", vars);
    count_ = 1;
    initialized_ = true;

    // algorithm: add timestamp
    // algorithm: add latency
    // algorithm: add message id
    // add latency measurements per message
    // calculate drops
  }
  else
  {
    ++count_;

    double latency =(double)(transport_context.get_current_time() -
      transport_context.get_message_time());

    latencies_.add(latency);
  }

  // count dropped messages
  if (no_apply_)
  {
    ++filtered_messages_;
    records.clear();
  }

  Originators::iterator originator_record = originators_.find(originator);

  // keep track of originators
  if (originator_record == originators_.end())
  {
    originators_[originator] = 1;
  }
  else
  {
    originator_record->second = originator_record->second + 1;
  }
}

size_t
filters::RcvCount::num_originators(void)
{
  return originators_.size();
}

filters::Originators
filters::RcvCount::originators(void)
{
  return originators_;
}
