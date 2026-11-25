// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_POSE_ARRAY_DESCRIPTOR_HPP
#define QRB_POSE_ARRAY_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

static_assert( std::is_trivially_copyable<Pose_t>::value,
               "Pose_t must be trivially copyable for DMA buffer storage" );
static_assert( std::is_standard_layout<Pose_t>::value,
               "Pose_t must be standard layout for DMA buffer storage" );

/**
 * @struct PoseArrayProps_t
 * @brief Allocation request for a robotics pose array buffer.
 *
 * Only what determines the memory size is kept here: the number of poses the buffer must be able
 * to hold. The number of poses actually stored is @c PoseArrayDescriptor_t::poseCount, set by the
 * producer after the buffer has been allocated.
 *
 * Inherited Members from QC::Memory::BufferProps_t:
 * @param size          Total required buffer size in bytes, set to
 *                      @c poseCount * sizeof( Pose_t ).
 * @param alignment     Alignment requirement of the buffer.
 * @param cache         Cache attributes of the buffer.
 * @param allocatorType Allocator used to allocate the buffer.
 *
 * New Members:
 * @param poseCount  Number of poses the buffer must be able to hold.
 *
 * @note Call @c UpdateSize() after changing @c poseCount directly.
 */
typedef struct PoseArrayProps : public QC::Memory::BufferProps_t
{
public:
    /**
     * @brief Default constructor. Creates an empty request with size 0.
     */
    PoseArrayProps() : QC::Memory::BufferProps_t(), poseCount( 0 ) {}

    ~PoseArrayProps() override = default;

    PoseArrayProps( const PoseArrayProps & ) = default;
    PoseArrayProps &operator=( const PoseArrayProps & ) = default;
    PoseArrayProps( PoseArrayProps && ) = default;
    PoseArrayProps &operator=( PoseArrayProps && ) = default;

    /**
     * @brief Constructor for PoseArrayProps.
     * @param count  Number of poses the buffer must be able to hold.
     * @param requestedAllocatorType  The allocator type, default is QC_MEMORY_ALLOCATOR_DMA.
     * @param requestedCache  The cache attributes, default is QC_CACHEABLE.
     * @note @c size is set to @c count * sizeof( Pose_t ).
     */
    explicit PoseArrayProps( uint32_t count,
                             QC::Memory::QCMemoryAllocator_e requestedAllocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA,
                             QC::Memory::QCAllocationCache_e requestedCache = QC::Memory::QC_CACHEABLE )
        : QC::Memory::BufferProps_t( 0, requestedAllocatorType, requestedCache ),
          poseCount( count )
    {
        UpdateSize();
    }

    /**
     * @brief Sets @c size to @c poseCount * sizeof( Pose_t ).
     */
    void UpdateSize()
    {
        size = static_cast<size_t>( poseCount ) * sizeof( Pose_t );
    }

    uint32_t poseCount;   /**< Number of poses the buffer must be able to hold. */

} PoseArrayProps_t;

/**
 * @struct PoseArrayDescriptor_t
 * @brief Descriptor for a robotics pose array buffer, a list of poses in one frame.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a list of poses packed into a buffer as a
 * contiguous array of @c Pose_t, for example the grasp candidates of a grasp planner or the
 * waypoints of a path. The layout mirrors ROS 2 @c geometry_msgs/PoseArray: one stamp and one
 * frame for the whole list.
 *
 * Buffer layout:
 * @code
 *   pBuf → [ Pose_t[0] | Pose_t[1] | ... | Pose_t[poseCount-1] ]
 * @endcode
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the buffer.
 * @param size          Total allocated size of the buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_1 for pose).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid pose data currently stored.
 * @param offset        Byte offset of the valid data within the buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs  Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                     matching ROS 2 std_msgs/Header.stamp.
 * @param frameId      Coordinate frame in which all poses are expressed
 *                     (e.g. "base_link"), matching ROS 2 std_msgs/Header.frame_id. A
 *                     @c FrameId_t, a fixed-capacity string of at most QRB_FRAME_ID_MAX_LEN
 *                     characters.
 * @param poseCount    Number of valid @c Pose_t entries in the buffer.
 *
 * @note The descriptor has no dynamic length member (no std::string / std::vector), so copying it
 * never allocates, as required for buffer descriptors. The poses themselves are stored in the
 * buffer as fixed-size POD structs.
 */
typedef struct PoseArrayDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    PoseArrayDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 ), poseCount( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_1;
    }

    ~PoseArrayDescriptor() override = default;

    PoseArrayDescriptor( const PoseArrayDescriptor &other ) = default;
    PoseArrayDescriptor( PoseArrayDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; pose-array-specific fields are left unchanged so the caller
     * can set them afterwards.
     */
    PoseArrayDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from another @c PoseArrayDescriptor_t.
     */
    PoseArrayDescriptor &operator=( const PoseArrayDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c PoseArrayDescriptor_t.
     */
    PoseArrayDescriptor &operator=( PoseArrayDescriptor &&other ) noexcept;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    PoseArrayDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Returns a pointer to the @p index -th pose in the buffer.
     *
     * The pointer is only as aligned as @c pBuf + @c offset is; a buffer whose offset is not a
     * multiple of @c alignof( Pose_t ) must be read and written with @c memcpy.
     *
     * @param index  Zero-based pose index.
     * @return Pointer to the requested @c Pose_t, or @c nullptr if @c pBuf is @c nullptr or
     *         @p index is not smaller than @c poseCount.
     */
    Pose_t *GetPosePtr( uint32_t index ) const
    {
        Pose_t *pPose = nullptr;
        if ( ( pBuf != nullptr ) && ( index < poseCount ) )
        {
            pPose = reinterpret_cast<Pose_t *>( GetDataPtr() ) + index;
        }
        return pPose;
    }

    /**
     * @brief Returns the minimum buffer size required to hold @p count poses.
     *
     * @param count  Number of poses to accommodate.
     * @return Required buffer size in bytes.
     */
    static size_t CalcBufferSize( uint32_t count )
    {
        return static_cast<size_t>( count ) * sizeof( Pose_t );
    }

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t  timestampNs;   /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t frameId;       /**< Coordinate frame identifier of all poses. */
    uint32_t  poseCount;     /**< Number of valid poses in the buffer. */

} PoseArrayDescriptor_t;

inline PoseArrayDescriptor &PoseArrayDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_1;
    return *this;
}

inline PoseArrayDescriptor &PoseArrayDescriptor::operator=( const PoseArrayDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_1;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->poseCount = other.poseCount;
    }
    return *this;
}

inline PoseArrayDescriptor &PoseArrayDescriptor::operator=( PoseArrayDescriptor &&other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_1;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->poseCount = other.poseCount;
    }
    return *this;
}

inline PoseArrayDescriptor &PoseArrayDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const PoseArrayDescriptor_t *pDesc = dynamic_cast<const PoseArrayDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pDesc )
        {
            PoseArrayDescriptor::operator=( *pDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            PoseArrayDescriptor::operator=( *pBufDesc );
        }
        else
        {
            QC::Memory::BufferDescriptor_t::operator=( other );
        }
    }
    return *this;
}

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_POSE_ARRAY_DESCRIPTOR_HPP
