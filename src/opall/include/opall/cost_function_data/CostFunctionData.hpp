#pragma once

#include <memory>

#include "opall/CostFunctionIdentifier.hpp"
#include "opall/Problem.hpp"
#include "opall/cost_function_data_id_generator.hpp"
#include "opall/evaluators.hpp"

namespace opall::cost_function_data
{
namespace details
{
class CostFunctionDataConcept
{
  public:
    virtual ~CostFunctionDataConcept() = default;
    virtual std::unique_ptr<CostFunctionDataConcept> clone() const = 0;
    virtual EvaluationResult evaluate() const = 0;
    virtual CostFunctionIdentifier generateIdentifier() const = 0;
    virtual void insertToProblem(opall::Problem &problem) const = 0;
};

template <typename CostFunctionDataConceptT, typename CostFunctionEvaluatorT>
class OwningCostFunctionDataModel : public CostFunctionDataConcept
{
  public:
    explicit OwningCostFunctionDataModel(CostFunctionDataConceptT &&costFunctionData, CostFunctionEvaluatorT &&evaluator)
        : m_costFunctionData{std::forward<CostFunctionDataConceptT>(costFunctionData)}, m_evaluator{std::forward<CostFunctionEvaluatorT>(evaluator)}
    {
    }

    EvaluationResult evaluate() const override
    {
        return m_evaluator(m_costFunctionData);
    }

    void insertToProblem(opall::Problem &problem) const override
    {
        problem.insertCostFunctionToProblem(m_costFunctionData);
    }

    CostFunctionIdentifier generateIdentifier() const override
    {
        return cost_function_id_generator::generate(m_costFunctionData);
    }

    std::unique_ptr<CostFunctionDataConcept> clone() const override
    {
        return std::make_unique<OwningCostFunctionDataModel>(*this);
    }

  private:
    CostFunctionDataConceptT m_costFunctionData;
    CostFunctionEvaluatorT m_evaluator;
};
} // namespace details

class CostFunctionData
{
  public:
    template <typename CostFunctionDataConceptT, typename CostFunctionEvaluatorT>
    CostFunctionData(CostFunctionDataConceptT &&costFunctionData, CostFunctionEvaluatorT &&costFunctionEvaluator)
    {
        using Model = details::OwningCostFunctionDataModel<CostFunctionDataConceptT, CostFunctionEvaluatorT>;
        m_pimpl = std::make_unique<Model>(std::forward<CostFunctionDataConceptT>(costFunctionData),
                                          std::forward<CostFunctionEvaluatorT>(costFunctionEvaluator));
    }

    CostFunctionData(const CostFunctionData &other) : m_pimpl(other.m_pimpl->clone())
    {
    }

    CostFunctionData &operator=(const CostFunctionData &other)
    {
        CostFunctionData copy{other};
        m_pimpl.swap(copy.m_pimpl);
        return *this;
    }

    ~CostFunctionData() = default;
    CostFunctionData(CostFunctionData &&) = default;
    CostFunctionData &operator=(CostFunctionData &&) = default;

  private:
    friend opall::EvaluationResult evaluate(const CostFunctionData &costFunctionData)
    {
        return costFunctionData.m_pimpl->evaluate();
    }

    friend void insertCostFunctionToProblem(const CostFunctionData &costFunctionData, opall::Problem &problem)
    {
        costFunctionData.m_pimpl->insertToProblem(problem);
    }

    friend CostFunctionIdentifier generateIdentifier(const CostFunctionData &costFunctionData)
    {
        return costFunctionData.m_pimpl->generateIdentifier();
    }

    std::unique_ptr<details::CostFunctionDataConcept> m_pimpl;
};

opall::EvaluationResult evaluate(const CostFunctionData &costFunctionData);
void insertCostFunctionToProblem(const CostFunctionData &costFunctionData, opall::Problem &problem);
CostFunctionIdentifier generateIdentifier(const CostFunctionData &costFunctionData);

} // namespace opall::cost_function_data