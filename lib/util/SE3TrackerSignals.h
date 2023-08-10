#pragma once
#include <type_traits>
#include <boost/signals2/signal.hpp>
#include <opencv2/core/mat.hpp>
#include <mutex>

namespace lsd_slam::util
{

class SE3TrackerSignals
{
public:
    
    using OnSignalConnectedType=boost::signals2::signal<void(const SE3TrackerSignals* pOwnerOfSignal,const boost::signals2::signal_base* pToSignal,bool)>;
    
//    explicit SE3TrackerSignals(OnSignalConnectedType* pOnConnectedSignal=nullptr);
    using OnCalcResidualStartedSignal=boost::signals2::signal<void(cv::Mat*& /*pMatOutput*/,const cv::Size&,std::recursive_mutex& /*mtx*/)>;
    using OnCalcResidualFinishedSignal=boost::signals2::signal<void(cv::Mat*& /*pMatOutput*/,std::recursive_mutex& /*mtx*/)>;
    using OnCalcResidualErrorCalculatedSignal=boost::signals2::signal<void(double /*error*/,int /*lvl*/,int /*iter*/)>;

    using OnCalcResidualAndBuffersDebugStart=boost::signals2::signal<void(const cv::Size&,std::mutex&,
                                                                    cv::Mat*&  ,
                                                                    cv::Mat*&  ,
                                                                    cv::Mat*&   )>;
    using OnCalcResidualAndBuffersDebugFinish=boost::signals2::signal<void( int     /*w              */     ,
                                                                            int     /*loop           */     ,
                                                                            int     /*buf_warped_size*/     ,
                                                                            int     /*goodCount      */     ,
                                                                            int     /*badCount       */     ,
                                                                            float   /*ratio          */     )>;

    template<typename TOwner,typename SignalReturn,typename Invokable,typename...args> 
    static void Connect(TOwner& instance,boost::signals2::signal<SignalReturn(args...)> TOwner::*pToSignalMember,Invokable invokable,OnSignalConnectedType* pConnectedSignal=nullptr)
    {
        (instance.*pToSignalMember).connect(invokable);
        if(pConnectedSignal!=nullptr && !pConnectedSignal->empty())
        {
            SE3TrackerSignals& root=static_cast<SE3TrackerSignals&>(instance);
            (*pConnectedSignal)(&root,static_cast<boost::signals2::signal_base*>(&(instance.*pToSignalMember)),true);
        }
    }

    template<typename TOwner,typename SignalReturn,typename...args> 
    static bool ConnectChain(TOwner& instance,boost::signals2::signal<SignalReturn(args...)> TOwner::*pToSignalMember,const boost::signals2::signal_base* pConnectedSignal,boost::signals2::signal<SignalReturn(args...)>& nextSignal)
    {
        auto& targetSignal= instance.*pToSignalMember;
        auto* pToSignal=static_cast<boost::signals2::signal_base*>(&(targetSignal));
        if(pToSignal == pConnectedSignal)
        {
            //if(nextSignal.empty())
            //{
            //    return true;
            //}
            nextSignal.connect(targetSignal);
            return true;
        }
        return false;
    }

    //OnSignalConnectedType& getOnSignalConnected();
//
    //std::unique_ptr< OnSignalConnectedType> _pOnSignalConnected;
    //OnSignalConnectedType OnSignalConnected;
};

#define CHECK_CONNECT_THIS_CHAIN(SignalName,psig,ref_to_next)        \
        if(SE3TrackerSignals::ConnectChain(*this,& std::decay_t<std::remove_pointer_t<decltype(this)>>::SignalName ,psig,ref_to_next)) \
        { \
            return; \
        } \

#define CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(SignalName,psig,ref_to_next)        \
        if(SE3TrackerSignals::ConnectChain(*this,& std::decay_t<std::remove_pointer_t<decltype(this)>>::SignalName ,psig,(ref_to_next).SignalName)) \
        { \
            return; \
        } \


}